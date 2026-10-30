// Same-LAN IPv4/UDP runtime destination + two independent expiring ARP endpoint caches.
// RX mailbox is immutable until acknowledged; only validated small frames enter it.
module network_runtime_dual #(
    parameter LOCAL_IP=32'hc0a8321e,LOCAL_MAC=48'h020a3501fec0,
    parameter NETMASK=32'hffffff00,DEFAULT_IP=32'hc0a8320a,
    parameter DEFAULT_PORT=16'd6102,VIDEO_SOURCE_PORT=16'd5000,CONTROL_PORT=16'd6200,
    parameter CLK_HZ=125000000,CACHE_SECONDS=30,
    parameter VISION_ENABLE=0,VISION_PORT=16'd6201,
    parameter EVENT_ENABLE=0,EVENT_PORT=16'd6202
)(input clk,rst_n,input rx_clk,rx_rst_n,input [7:0] rx_data,input rx_valid,rx_error,
    input video_start,video_first,input [15:0] video_length,input [7:0] video_data,
    output video_read,output reg video_done,
    output reg [7:0] tx_data,output reg tx_valid,
    output reg [31:0] target_ip,output reg [15:0] target_port,
    output cache_hit,output [47:0] target_mac,output reg enabled,
    output reg [31:0] arp_requests,rx_packets,output [31:0] rx_debug,
    output reg [31:0] target2_ip,output reg [15:0] target2_port,
    output reg enabled2,output cache2_hit,output [47:0] target2_mac,
    output reg vision_valid,output reg [255:0] vision_payload,
    input [255:0] vision_status,input vision_status_valid,
    input event_valid,input [255:0] event_payload,output event_ready);

    function [31:0] crc_byte;
        input [31:0] crc;input [7:0] value;integer k;reg [31:0] c;
        begin c=crc^value;for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end
    endfunction
    function host_ip;
        input [31:0] ip;
        begin host_ip=(ip&NETMASK)==(LOCAL_IP&NETMASK) && ip!=LOCAL_IP &&
            (ip&~NETMASK)!=0 && (ip&~NETMASK)!=(~NETMASK);end
    endfunction

    localparam RX_BYTES=VISION_ENABLE ? 80 : 64;
    reg [7:0] memory[0:RX_BYTES-1];
    reg [RX_BYTES-1:0] write_mask;reg [7:0] write_byte;integer w,v;
    reg request_toggle,ack_toggle,ack_sync0,ack_sync1,req_sync0,req_sync1;
    reg [7:0] frame_length,rx_count;
    reg [3:0] preamble;
    reg receiving,reject_frame,rx_active;
    reg [31:0] rx_crc;
    reg [7:0] rx_frames,rx_good,first_sample,last_length;
    reg [15:0] arp_debug;
    // Isolate IDDR recovery from preamble/CRC/mailbox control fanout.
    reg [7:0] rx_byte;reg rx_byte_valid,rx_byte_error;
    always @(posedge rx_clk or negedge rx_rst_n)begin
        if(!rx_rst_n)begin rx_byte<=0;rx_byte_valid<=0;rx_byte_error<=0;end
        else begin rx_byte<=rx_data;rx_byte_valid<=rx_valid;rx_byte_error<=rx_error;end
    end
    assign rx_debug={rx_frames,rx_good,arp_debug};
    always @(posedge rx_clk or negedge rx_rst_n)begin
        if(!rx_rst_n)begin
            ack_sync0<=0;ack_sync1<=0;request_toggle<=0;frame_length<=0;
            rx_count<=0;preamble<=0;receiving<=0;reject_frame<=0;rx_active<=0;rx_crc<=32'hffffffff;
            write_mask<=0;write_byte<=0;
            rx_frames<=0;rx_good<=0;first_sample<=0;last_length<=0;
        end else begin
            write_mask<=0;write_byte<=rx_byte;
            for(w=0;w<RX_BYTES;w=w+1)if(write_mask[w])memory[w]<=write_byte;
            ack_sync0<=ack_toggle;ack_sync1<=ack_sync0;rx_active<=rx_byte_valid;
            if(rx_byte_valid)begin
                if(!rx_active)begin
                    preamble<=rx_byte==8'h55 ? 1 : 0;receiving<=0;rx_count<=0;
                    first_sample<=rx_byte;
                    reject_frame<=request_toggle!=ack_sync1 || rx_byte_error;rx_crc<=32'hffffffff;
                end else if(!receiving)begin
                    if(first_sample==8'h55 && rx_byte!=8'h55)first_sample<=rx_byte;
                    if(rx_byte==8'h55 && preamble<7)preamble<=preamble+1'b1;
                    else if(rx_byte==8'hd5 && preamble==7)receiving<=1;
                    else reject_frame<=1;
                end else begin
                    if(rx_count<RX_BYTES && !reject_frame)write_mask<={{(RX_BYTES-1){1'b0}},1'b1}<<rx_count[6:0];
                    if(rx_count<255)rx_count<=rx_count+1'b1;
                    if(rx_count>=128 || rx_byte_error)reject_frame<=1;
                    rx_crc<=crc_byte(rx_crc,rx_byte);
                end
            end else if(rx_active)begin
                rx_frames<=rx_frames+1'b1;last_length<=rx_count;
                if(receiving && !reject_frame && rx_count>=64 && rx_count<=128 && rx_crc==32'hdebb20e3)begin
                    frame_length<=rx_count;request_toggle<=~request_toggle;
                    rx_good<=rx_good+1'b1;
                end
                receiving<=0;preamble<=0;
            end
        end
    end
    wire [47:0] peer_mac={memory[6],memory[7],memory[8],memory[9],memory[10],memory[11]};
    wire [47:0] dest_mac={memory[0],memory[1],memory[2],memory[3],memory[4],memory[5]};
    wire [15:0] eth_type={memory[12],memory[13]};
    wire [31:0] arp_ip={memory[28],memory[29],memory[30],memory[31]};
    wire [47:0] arp_mac={memory[22],memory[23],memory[24],memory[25],memory[26],memory[27]};
    wire [47:0] arp_tha={memory[32],memory[33],memory[34],memory[35],memory[36],memory[37]};
    wire [31:0] arp_tpa={memory[38],memory[39],memory[40],memory[41]};
    wire [31:0] peer_ip={memory[26],memory[27],memory[28],memory[29]};
    wire [31:0] udp_dest_ip={memory[30],memory[31],memory[32],memory[33]};
    wire [15:0] ip_length={memory[16],memory[17]},udp_length={memory[38],memory[39]};
    wire [31:0] requested_ip={memory[54],memory[55],memory[56],memory[57]};
    wire [15:0] requested_port={memory[58],memory[59]};
    wire [15:0] peer_port={memory[34],memory[35]};
    wire [31:0] request_id={memory[50],memory[51],memory[52],memory[53]};
    wire peer_valid=peer_mac!=0 && !peer_mac[40];
    reg arp_accept;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)arp_accept<=0;
        else arp_accept<=peer_valid && (dest_mac==LOCAL_MAC || dest_mac==48'hffffffffffff) && eth_type==16'h0806 &&
            {memory[14],memory[15],memory[16],memory[17],memory[18],memory[19]}==48'h000108000604 &&
            memory[20]==0 && (memory[21]==1 || memory[21]==2) && arp_mac==peer_mac && arp_tpa==LOCAL_IP &&
            (memory[21]==1 || arp_tha==LOCAL_MAC);
    end
    // One tagged ARP entry per endpoint: .100 and .20 must not collide by low IP bits.
    reg [31:0] cache_ip0,cache_ip1;
    reg [47:0] cache_mac0,cache_mac1,found_mac,found2_mac;
    reg [7:0] cache_age0,cache_age1;
    reg cache_valid0,cache_valid1,lookup_valid,lookup2_valid,refresh_due,refresh2_due;
    reg clear_cache,clear_cache2;
    reg learn_pending,learn_match0,learn_match1;
    wire found=lookup_valid && !clear_cache;
    wire found2=lookup2_valid && !clear_cache2;
    wire found_comb=cache_valid0 && cache_ip0==target_ip;
    wire found2_comb=cache_valid1 && cache_ip1==target2_ip;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin lookup_valid<=0;lookup2_valid<=0;refresh_due<=0;refresh2_due<=0;found_mac<=0;found2_mac<=0;end
        else begin
            lookup_valid<=found_comb;lookup2_valid<=found2_comb;
            found_mac<=cache_mac0;found2_mac<=cache_mac1;
            refresh_due<=found_comb && cache_age0<=(CACHE_SECONDS>5?5:1);
            refresh2_due<=found2_comb && cache_age1<=(CACHE_SECONDS>5?5:1);
            if(clear_cache || (parse_state==P_COMMAND && (memory[47]==2 || memory[47]==4)))lookup_valid<=0;
            if(clear_cache2 || (parse_state==P_COMMAND && (memory[47]==5 || memory[47]==8)))lookup2_valid<=0;
        end
    end
    assign cache_hit=found;assign target_mac=found_mac;
    assign cache2_hit=found2;assign target2_mac=found2_mac;
    localparam SECOND_BITS=CLK_HZ<2 ? 1 : $clog2(CLK_HZ);
    reg [SECOND_BITS-1:0] second_count;reg second_tick;
    localparam RETRY_BITS=$clog2(CLK_HZ*2+1);
    reg [RETRY_BITS-1:0] retry_count,retry_count2;
    reg arp_due0,arp_due1;
    // Retry reductions must not sit in front of the full dispatch priority tree.
    // A registered request may be one clock old; dispatch rechecks availability.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin arp_due0<=0;arp_due1<=0;end
        else begin
            arp_due0<=enabled && (!found || refresh_due) && retry_count==0;
            arp_due1<=enabled2 && (!found2 || refresh2_due) && retry_count2==0;
        end
    end
    reg [2:0] attempts,attempts2;
    reg learn;reg [31:0] learned_ip;reg [47:0] learned_mac;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cache_valid0<=0;cache_valid1<=0;second_count<=0;second_tick<=CLK_HZ==1;
            cache_ip0<=0;cache_ip1<=0;cache_mac0<=0;cache_mac1<=0;cache_age0<=0;cache_age1<=0;
            learn_pending<=0;learn_match0<=0;learn_match1<=0;
        end else begin
            learn_pending<=learn;learn_match0<=learned_ip==target_ip;learn_match1<=learned_ip==target2_ip;
            // Predict the terminal count by a beat; cache expiry still occurs
            // on the original CLK_HZ-th edge rather than one cycle later.
            second_tick<=CLK_HZ==1 || (!second_tick && second_count==CLK_HZ-2);
            if(second_tick)begin
                second_count<=0;
                if(cache_valid0)begin if(cache_age0<=1)cache_valid0<=0;else cache_age0<=cache_age0-1'b1;end
                if(cache_valid1)begin if(cache_age1<=1)cache_valid1<=0;else cache_age1<=cache_age1-1'b1;end
            end else second_count<=second_count+1'b1;
            if(clear_cache)cache_valid0<=0;
            else if(learn_pending && learn_match0)begin cache_ip0<=learned_ip;cache_mac0<=learned_mac;cache_age0<=CACHE_SECONDS;cache_valid0<=1;end
            if(clear_cache2)cache_valid1<=0;
            else if(learn_pending && learn_match1)begin cache_ip1<=learned_ip;cache_mac1<=learned_mac;cache_age1<=CACHE_SECONDS;cache_valid1<=1;end
        end
    end

    // Sequential checksum scan, after FCS success and while RX mailbox is locked.
    localparam P_IDLE=0,P_IP=1,P_UDP=2,P_COMMAND=3,P_CLASSIFY=4,P_IP_WAIT=5,P_UDP_WAIT=6,
        P_VISION=7,P_VISION_WAIT=8,P_VISION_REPLY=9,P_IP_ADD=10,P_UDP_ADD=11;
    reg [3:0] parse_state;reg [6:0] scan_index;
    reg [15:0] scan_word;
    reg parsing_vision,vision_pending;
    reg [255:0] vision_reply;
    reg [47:0] vision_mac;reg [31:0] vision_ip;reg [15:0] vision_peer_port;
    reg control_shape,vision_shape,ip_envelope_ok,command_ok,requested_ok0,requested_ok1,vision_source_ok,video_length_ok;
    reg [5:0] vision_mac_equal;
    reg [3:0] vision_ip_equal;
    integer identity_index;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            control_shape<=0;vision_shape<=0;ip_envelope_ok<=0;command_ok<=0;requested_ok0<=0;requested_ok1<=0;
            vision_mac_equal<=0;vision_ip_equal<=0;vision_source_ok<=0;video_length_ok<=0;
        end else begin
            vision_source_ok<=vision_shape && enabled2 && found2 && (&vision_ip_equal) && (&vision_mac_equal);
            video_length_ok<=video_length<=1456;
            // Byte-sized equality stages avoid a 48-bit comparator on the parser enable path.
            for(identity_index=0;identity_index<6;identity_index=identity_index+1)
                vision_mac_equal[identity_index]<=peer_mac[identity_index*8 +: 8]==found2_mac[identity_index*8 +: 8];
            for(identity_index=0;identity_index<4;identity_index=identity_index+1)
                vision_ip_equal[identity_index]<=peer_ip[identity_index*8 +: 8]==target2_ip[identity_index*8 +: 8];
            control_shape<=ip_length==48 && udp_length==28 && frame_length==66 && {memory[36],memory[37]}==CONTROL_PORT;
            vision_shape<=ip_length==60 && udp_length==40 && frame_length==78 && {memory[36],memory[37]}==VISION_PORT;
            ip_envelope_ok<=peer_valid && dest_mac==LOCAL_MAC && eth_type==16'h0800 && memory[14]==8'h45 &&
                memory[23]==17 && ({memory[20],memory[21]}&16'hbfff)==0 && udp_dest_ip==LOCAL_IP && host_ip(peer_ip);
            command_ok<={memory[42],memory[43],memory[44],memory[45]}==32'h4e434647 && memory[46]==1 &&
                {memory[48],memory[49],memory[60],memory[61]}==0;
            requested_ok0<=host_ip(requested_ip) && requested_port!=0 && !(enabled2 && target2_ip==requested_ip && target2_port==requested_port);
            requested_ok1<=host_ip(requested_ip) && requested_port!=0 && !(enabled && target_ip==requested_ip && target_port==requested_port);
        end
    end
    wire control_datagram=control_shape && !control_pending && !ack_build && !(send_state==S_UDP && !sending_video);
    // Slot 1 is the semantic producer. ARP identity and enabled state must agree.
    wire vision_datagram=VISION_ENABLE && vision_source_ok;
    reg [20:0] check_sum;
    wire [16:0] checksum_fold={1'b0,check_sum[15:0]}+{12'd0,check_sum[20:16]};
    wire [16:0] checksum_end={1'b0,checksum_fold[15:0]}+{16'd0,checksum_fold[16]};
    reg checksum_ok;
    always @(posedge clk or negedge rst_n)if(!rst_n)checksum_ok<=0;else checksum_ok<=checksum_end==17'h0ffff;
    reg arp_reply_pending;reg [47:0] reply_mac;reg [31:0] reply_ip;
    reg control_second;reg control_pending,ack_build;reg [47:0] control_mac;reg [31:0] control_ip,control_id;
    reg [15:0] control_port;reg [7:0] control_opcode,control_error;
    reg [255:0] control_payload;
    reg [255:0] send_control_payload;
    reg event_pending;reg [255:0] event_buffer;
    assign event_ready=!event_pending;
    reg wait_first,wait_first2,video_pending,video_is_first;
    wire eligible0=enabled && found && (!wait_first || video_is_first);
    wire eligible1=enabled2 && found2 && (!wait_first2 || video_is_first);
    reg replay_pending,using_replay;
    reg [47:0] replay_mac;reg [31:0] replay_ip;reg [15:0] replay_port;
    // First send consumes source once; second send replays exactly the same immutable payload.
    reg [7:0] packet_ram[0:2047];reg [10:0] write_index,replay_index;reg [7:0] replay_word;
    wire [10:0] replay_prefetch=replay_index+(udp_read && using_replay);
    always @(posedge clk)begin
        if(video_read)packet_ram[write_index]<=video_data;
        replay_word<=packet_ram[replay_prefetch];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin write_index<=0;replay_index<=0;end
        else begin
            if(video_start)write_index<=0;else if(video_read)write_index<=write_index+1'b1;
            if(udp_start)replay_index<=0;else if(udp_read && using_replay)replay_index<=replay_index+1'b1;
        end
    end
    wire network_reset=~rst_n;
    reg [2:0] send_state;
    localparam S_IDLE=0,S_UDP=1,S_ARP=2,S_GAP=3,S_DISPATCH=4;
    localparam A_NONE=0,A_REPLY=1,A_CONTROL=2,A_VISION=3,A_ARP0=4,A_ARP1=5,A_VIDEO=6,A_EVENT=7;
    reg [2:0] action;
    reg udp_start,sending_video;reg [47:0] send_mac;reg [31:0] send_ip;
    reg [15:0] send_port,send_source_port,send_length;
    reg [5:0] control_index;
    wire udp_done,udp_read,udp_valid,udp_clock;wire [7:0] udp_data;
    wire [7:0] udp_payload=sending_video?(using_replay?replay_word:video_data):send_control_payload[255:248];
    eth_udp_tx_gmii udp(.clk125m(clk),.reset_p(network_reset),.tx_en_pulse(udp_start),.tx_done(udp_done),
        .dst_mac(send_mac),.src_mac(LOCAL_MAC),.dst_ip(send_ip),.src_ip(LOCAL_IP),
        .dst_port(send_port),.src_port(send_source_port),.data_length(send_length),
        .payload_req_o(udp_read),.payload_dat_i(udp_payload),.gmii_tx_clk(udp_clock),.gmii_txen(udp_valid),.gmii_txd(udp_data));
    assign video_read=udp_read && sending_video && !using_replay && send_state==S_UDP;

    // Complete minimum-sized Ethernet ARP frame: 8-byte preamble, 60-byte frame, 4-byte FCS.
    reg [6:0] arp_index;reg [31:0] arp_crc;
    reg [47:0] arp_send_mac,arp_target_mac;reg [31:0] arp_target_ip;reg [15:0] arp_opcode;
    reg [479:0] arp_body;
    reg [7:0] arp_byte;
    always @*begin
        arp_byte=0;
        if(arp_index<7)arp_byte=8'h55;
        else if(arp_index==7)arp_byte=8'hd5;
        else if(arp_index<68)arp_byte=arp_body[479:472];
        else case(arp_index)
            68:arp_byte=~arp_crc[7:0];69:arp_byte=~arp_crc[15:8];70:arp_byte=~arp_crc[23:16];71:arp_byte=~arp_crc[31:24];
            default:arp_byte=0;
        endcase
    end
    // udp_done precedes the final registered FCS bytes; retain its actual valid tail.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin tx_valid<=0;tx_data<=0;end
        else begin tx_valid<=send_state==S_ARP || udp_valid;tx_data<=send_state==S_ARP ? arp_byte : udp_data;end
    end
    reg [4:0] gap;

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            req_sync0<=0;req_sync1<=0;ack_toggle<=0;parse_state<=P_IDLE;scan_index<=0;check_sum<=0;
            target_ip<=DEFAULT_IP;target_port<=DEFAULT_PORT;enabled<=DEFAULT_IP!=0;
            target2_ip<=0;target2_port<=DEFAULT_PORT;enabled2<=0;wait_first2<=1;
            replay_pending<=0;using_replay<=0;replay_mac<=0;replay_ip<=0;replay_port<=0;control_second<=0;
            learn<=0;learned_ip<=0;learned_mac<=0;clear_cache<=0;clear_cache2<=0;
            reply_mac<=0;reply_ip<=0;arp_reply_pending<=0;control_pending<=0;ack_build<=0;
            control_mac<=0;control_ip<=0;control_id<=0;control_port<=0;control_opcode<=0;control_error<=0;control_payload<=0;arp_debug<=0;
            send_state<=S_IDLE;udp_start<=0;sending_video<=0;send_mac<=0;send_ip<=0;send_port<=0;send_source_port<=0;send_length<=0;
            control_index<=0;video_done<=0;video_pending<=0;video_is_first<=0;wait_first<=1;
            retry_count<=0;attempts<=0;retry_count2<=0;attempts2<=0;arp_requests<=0;rx_packets<=0;arp_index<=0;arp_crc<=32'hffffffff;
            arp_send_mac<=0;arp_target_mac<=0;arp_target_ip<=0;arp_opcode<=0;arp_body<=0;gap<=0;
            parsing_vision<=0;vision_valid<=0;vision_payload<=0;vision_pending<=0;send_control_payload<=0;
            vision_reply<=0;vision_mac<=0;vision_ip<=0;vision_peer_port<=0;
            scan_word<=0;action<=A_NONE;event_pending<=0;event_buffer<=0;
        end else begin
            req_sync0<=request_toggle;req_sync1<=req_sync0;learn<=0;clear_cache<=0;clear_cache2<=0;
            udp_start<=0;video_done<=0;vision_valid<=0;
            if(!enabled2 || !found2)event_pending<=0;
            if(EVENT_ENABLE && event_valid && event_ready && enabled2 && found2)begin event_buffer<=event_payload;event_pending<=1;end
            if(retry_count!=0)retry_count<=retry_count-1'b1;
            if(found && !refresh_due)begin attempts<=0;retry_count<=0;end
            if(retry_count2!=0)retry_count2<=retry_count2-1'b1;
            if(found2 && !refresh2_due)begin attempts2<=0;retry_count2<=0;end
            if(video_start)begin video_pending<=1;video_is_first<=video_first;end
            if(udp_read && !sending_video)control_index<=control_index+1'b1;
            if(udp_read && !sending_video)send_control_payload<={send_control_payload[247:0],8'd0};
            if(ack_build)begin
                control_payload<={32'h4e535441,8'd1,control_opcode|8'h80,control_error,
                    3'd0,(EVENT_ENABLE ? 1'b1 : 1'b0),control_second,1'b1,(control_second?found2:found),(control_second?enabled2:enabled),
                    control_id,(control_second?target2_ip:target_ip),(control_second?target2_port:target_port),
                    (control_second?found2_mac:found_mac),arp_requests,rx_packets};
                control_pending<=1;ack_build<=0;
            end
            case(parse_state)
                P_IDLE:if(req_sync1!=ack_toggle)begin parse_state<=P_CLASSIFY;end
                P_CLASSIFY:begin
                    rx_packets<=rx_packets+1'b1;
                    if(eth_type==16'h0806)arp_debug<={dest_mac==LOCAL_MAC || dest_mac==48'hffffffffffff,peer_valid,
                        {memory[14],memory[15],memory[16],memory[17],memory[18],memory[19]}==48'h000108000604,
                        arp_mac==peer_mac,arp_tpa==LOCAL_IP,memory[21]==1,arp_tha==LOCAL_MAC,1'b0,arp_tpa[7:0]};
                    if(arp_accept)begin
                        if(host_ip(arp_ip))begin learn<=1;learned_ip<=arp_ip;learned_mac<=arp_mac;end
                        if(memory[21]==1 && !arp_reply_pending)begin arp_reply_pending<=1;reply_ip<=arp_ip;reply_mac<=arp_mac;end
                        ack_toggle<=req_sync1;parse_state<=P_IDLE;
                    end else if(ip_envelope_ok &&
                        (control_datagram || vision_datagram))begin
                        parsing_vision<=vision_datagram;check_sum<=0;scan_index<=14;parse_state<=P_IP;
                    end else begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                end
                P_IP:if(scan_index<34)begin
                    scan_word<={memory[scan_index],memory[scan_index+7'd1]};parse_state<=P_IP_ADD;
                end else parse_state<=P_IP_WAIT;
                P_IP_ADD:begin check_sum<=check_sum+scan_word;scan_index<=scan_index+7'd2;parse_state<=P_IP;end
                P_IP_WAIT:if(!checksum_ok)begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                else if({memory[40],memory[41]}==0)parse_state<=parsing_vision ? P_VISION : P_COMMAND;
                else begin
                    check_sum<=peer_ip[31:16]+{16'd0,peer_ip[15:0]}+{16'd0,LOCAL_IP[31:16]}+{16'd0,LOCAL_IP[15:0]}+32'd17+{16'd0,udp_length};
                    scan_index<=34;parse_state<=P_UDP;
                end
                P_UDP:if(scan_index<(parsing_vision ? 74 : 62))begin
                    scan_word<={memory[scan_index],memory[scan_index+7'd1]};parse_state<=P_UDP_ADD;
                end else parse_state<=P_UDP_WAIT;
                P_UDP_ADD:begin check_sum<=check_sum+scan_word;scan_index<=scan_index+7'd2;parse_state<=P_UDP;end
                P_UDP_WAIT:if(checksum_ok)parse_state<=parsing_vision ? P_VISION : P_COMMAND;
                else begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                P_VISION:begin
                    for(v=0;v<32;v=v+1)vision_payload[255-v*8 -: 8]<=memory[42+v];
                    vision_valid<=1;parse_state<=P_VISION_WAIT;
                end
                P_VISION_WAIT:if(vision_status_valid)parse_state<=P_VISION_REPLY;
                P_VISION_REPLY:begin
                    // Best-effort reply; a busy reply slot never blocks result state or video.
                    if(!vision_pending)begin
                        vision_reply<=vision_status;vision_mac<=peer_mac;vision_ip<=peer_ip;
                        vision_peer_port<=peer_port;vision_pending<=1;
                    end
                    ack_toggle<=req_sync1;parse_state<=P_IDLE;
                end
                P_COMMAND:begin
                    ack_toggle<=req_sync1;parse_state<=P_IDLE;
                    if(command_ok)begin
                        control_mac<=peer_mac;control_ip<=peer_ip;control_port<=peer_port;control_id<=request_id;
                        control_opcode<=memory[47];control_second<=memory[47]>=5 && memory[47]<=8;control_error<=0;ack_build<=1;
                        case(memory[47])
                            1:begin end
                            2:if(requested_ok0)begin
                                target_ip<=requested_ip;target_port<=requested_port;enabled<=1;wait_first<=1;retry_count<=0;attempts<=0;clear_cache<=1;
                            end else control_error<=1;
                            3:begin enabled<=0;wait_first<=1;end
                            4:begin clear_cache<=1;wait_first<=1;retry_count<=0;attempts<=0;end
                            5:if(requested_ok1)begin
                                target2_ip<=requested_ip;target2_port<=requested_port;enabled2<=1;wait_first2<=1;retry_count2<=0;attempts2<=0;clear_cache2<=1;
                            end else control_error<=1;
                            6:begin enabled2<=0;wait_first2<=1;end
                            7:begin end
                            8:begin clear_cache2<=1;wait_first2<=1;retry_count2<=0;attempts2<=0;end
                            default:control_error<=2;
                        endcase
                    end
                end
            endcase
            case(send_state)
                S_IDLE:begin
                    action<=arp_reply_pending ? A_REPLY : control_pending ? A_CONTROL : vision_pending ? A_VISION :
                        EVENT_ENABLE && event_pending && enabled2 && found2 ? A_EVENT :
                        arp_due0 ? A_ARP0 : arp_due1 ? A_ARP1 : video_pending ? A_VIDEO : A_NONE;
                    send_state<=S_DISPATCH;
                end
                S_DISPATCH:begin
                    send_state<=S_IDLE;
                    case(action)
                    A_REPLY:begin
                        arp_send_mac<=reply_mac;arp_target_mac<=reply_mac;arp_target_ip<=reply_ip;arp_opcode<=2;
                        arp_body<={reply_mac,LOCAL_MAC,16'h0806,16'h0001,16'h0800,8'd6,8'd4,16'd2,LOCAL_MAC,LOCAL_IP,reply_mac,reply_ip,144'd0};
                        arp_reply_pending<=0;arp_index<=0;arp_crc<=32'hffffffff;send_state<=S_ARP;
                    end
                    A_CONTROL:begin
                        send_mac<=control_mac;send_ip<=control_ip;send_port<=control_port;send_source_port<=CONTROL_PORT;send_length<=32;
                        send_control_payload<=control_payload;sending_video<=0;control_index<=0;control_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end
                    A_VISION:begin
                        send_mac<=vision_mac;send_ip<=vision_ip;send_port<=vision_peer_port;send_source_port<=VISION_PORT;send_length<=32;
                        send_control_payload<=vision_reply;sending_video<=0;control_index<=0;vision_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end
                    A_EVENT:begin
                        event_pending<=0;
                        // Snapshot data unconditionally; only a valid destination
                        // permits udp_start. This removes wide destination enables.
                        send_mac<=found2_mac;send_ip<=target2_ip;send_port<=EVENT_PORT;send_source_port<=EVENT_PORT;send_length<=32;
                        send_control_payload<=event_buffer;sending_video<=0;control_index<=0;
                        if(enabled2 && found2)begin
                            udp_start<=1;send_state<=S_UDP;
                        end
                    end
                    A_ARP0:if(arp_due0 && enabled && (!found || refresh_due))begin
                        arp_send_mac<=48'hffffffffffff;arp_target_mac<=0;arp_target_ip<=target_ip;arp_opcode<=1;
                        arp_body<={48'hffffffffffff,LOCAL_MAC,16'h0806,16'h0001,16'h0800,8'd6,8'd4,16'd1,LOCAL_MAC,LOCAL_IP,48'd0,target_ip,144'd0};
                        arp_index<=0;arp_crc<=32'hffffffff;send_state<=S_ARP;arp_requests<=arp_requests+1'b1;
                        retry_count<=attempts<3?CLK_HZ/2:CLK_HZ*2;if(attempts<3)attempts<=attempts+1'b1;
                    end
                    A_ARP1:if(arp_due1 && enabled2 && (!found2 || refresh2_due))begin
                        arp_send_mac<=48'hffffffffffff;arp_target_mac<=0;arp_target_ip<=target2_ip;arp_opcode<=1;
                        arp_body<={48'hffffffffffff,LOCAL_MAC,16'h0806,16'h0001,16'h0800,8'd6,8'd4,16'd1,LOCAL_MAC,LOCAL_IP,48'd0,target2_ip,144'd0};
                        arp_index<=0;arp_crc<=32'hffffffff;send_state<=S_ARP;arp_requests<=arp_requests+1'b1;
                        retry_count2<=attempts2<3?CLK_HZ/2:CLK_HZ*2;if(attempts2<3)attempts2<=attempts2+1'b1;
                    end
                    A_VIDEO:begin
                        video_pending<=0;
                        if(!found)wait_first<=1;
                        if(!found2)wait_first2<=1;
                        if((!eligible0 && !eligible1) || !video_length_ok)begin video_done<=1;end
                        else begin
                            if(eligible0)wait_first<=0;
                            if(eligible1)wait_first2<=0;
                            send_mac<=eligible0?found_mac:found2_mac;
                            send_ip<=eligible0?target_ip:target2_ip;
                            send_port<=eligible0?target_port:target2_port;send_source_port<=VIDEO_SOURCE_PORT;
                            replay_pending<=eligible0 && eligible1;using_replay<=0;
                            replay_mac<=found2_mac;replay_ip<=target2_ip;replay_port<=target2_port;
                            send_length<=video_length;sending_video<=1;udp_start<=1;send_state<=S_UDP;
                        end
                    end
                    default:begin end
                    endcase
                end
                S_UDP:if(udp_done)begin
                    if(sending_video && !replay_pending)video_done<=1;send_state<=S_GAP;gap<=0;
                end
                S_ARP:begin
                    if(arp_index>=8 && arp_index<68)begin arp_crc<=crc_byte(arp_crc,arp_byte);arp_body<={arp_body[471:0],8'd0};end
                    if(arp_index==71)begin send_state<=S_GAP;gap<=0;end else arp_index<=arp_index+1'b1;
                end
                S_GAP:if(gap==15)begin
                    if(replay_pending)begin
                        send_mac<=replay_mac;send_ip<=replay_ip;send_port<=replay_port;
                        using_replay<=1;replay_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end else send_state<=S_IDLE;
                end else gap<=gap+1'b1;
            endcase
        end
    end
endmodule
