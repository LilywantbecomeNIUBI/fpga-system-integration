// Same-LAN IPv4/UDP runtime destination + two independent expiring ARP endpoint caches.
// RX mailbox is immutable until acknowledged; only validated small frames enter it.
module network_runtime_dual #(
    parameter LOCAL_IP=32'hc0a8321e,LOCAL_MAC=48'h020a3501fec0,
    parameter NETMASK=32'hffffff00,DEFAULT_IP=32'hc0a8320a,
    parameter DEFAULT_PORT=16'd6102,VIDEO_SOURCE_PORT=16'd5000,CONTROL_PORT=16'd6200,
    parameter CLK_HZ=125000000,CACHE_SECONDS=30,
    parameter VISION_ENABLE=0,VISION_PORT=16'd6201,
    parameter EVENT_ENABLE=0,EVENT_PORT=16'd6202,
    parameter TELEMETRY_MIRROR_PRIMARY=0,
    parameter MOTOR_ENABLE=0,MOTOR_PORT=16'd6206,
    parameter TELEMETRY_ENABLE=0,TELEMETRY_PORT=16'd6205
)(input clk,rst_n,input rx_clk,rx_rst_n,input [7:0] rx_data,input rx_valid,rx_error,
    input video_start,video_first,input [15:0] video_length,input [7:0] video_data,
    output video_read,output reg video_done,
    output reg [7:0] tx_data,output reg tx_valid,
    (* syn_preserve=1, syn_keep=1 *) output reg [31:0] target_ip,output reg [15:0] target_port,
    output cache_hit,output [47:0] target_mac,output reg enabled,
    output reg [31:0] arp_requests,rx_packets,output [31:0] rx_debug,
    (* syn_preserve=1, syn_keep=1 *) output reg [31:0] target2_ip,output reg [15:0] target2_port,
    output reg enabled2,output cache2_hit,output [47:0] target2_mac,
    output reg vision_valid,output reg [255:0] vision_payload,
    input [255:0] vision_status,input vision_status_valid,
    input event_valid,input [255:0] event_payload,output event_ready,
    input telemetry_valid,input [511:0] telemetry_payload,output telemetry_ready,
    output reg motor_valid,output reg [191:0] motor_frame,input motor_ready);

    function [31:0] crc_byte;
        input [31:0] crc;input [7:0] value;integer k;reg [31:0] c;
        begin c=crc^value;for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end
    endfunction
    function host_ip;
        input [31:0] ip;
        begin host_ip=(ip&NETMASK)==(LOCAL_IP&NETMASK) && ip!=LOCAL_IP &&
            (ip&~NETMASK)!=0 && (ip&~NETMASK)!=(~NETMASK);end
    endfunction

    localparam RX_BYTES=(VISION_ENABLE || TELEMETRY_ENABLE || MOTOR_ENABLE) ? 80 : 64;
    reg [7:0] memory[0:RX_BYTES-1];
    reg [RX_BYTES-1:0] write_mask;reg [7:0] write_byte;integer w,v;
    reg request_toggle,ack_toggle,ack_sync0,ack_sync1,req_sync0,req_sync1;
    reg [7:0] frame_length,rx_count;
    reg [3:0] preamble;
    reg receiving,reject_frame,rx_active,length_in_range,rx_count_full;
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
            rx_count<=0;preamble<=0;receiving<=0;reject_frame<=0;rx_active<=0;length_in_range<=0;rx_count_full<=0;rx_crc<=32'hffffffff;
            write_mask<=0;write_byte<=0;
            rx_frames<=0;rx_good<=0;first_sample<=0;last_length<=0;
        end else begin
            write_mask<=0;write_byte<=rx_byte;
            for(w=0;w<RX_BYTES;w=w+1)if(write_mask[w])memory[w]<=write_byte;
            ack_sync0<=ack_toggle;ack_sync1<=ack_sync0;rx_active<=rx_byte_valid;
            if(rx_byte_valid)begin
                if(!rx_active)begin
                    preamble<=rx_byte==8'h55 ? 1 : 0;receiving<=0;rx_count<=0;length_in_range<=0;rx_count_full<=0;
                    first_sample<=rx_byte;
                    reject_frame<=request_toggle!=ack_sync1 || rx_byte_error;rx_crc<=32'hffffffff;
                end else if(!receiving)begin
                    if(first_sample==8'h55 && rx_byte!=8'h55)first_sample<=rx_byte;
                    if(rx_byte==8'h55 && preamble<7)preamble<=preamble+1'b1;
                    else if(rx_byte==8'hd5 && preamble==7)receiving<=1;
                    else reject_frame<=1;
                end else begin
                    if(rx_count<RX_BYTES && !reject_frame)write_mask<={{(RX_BYTES-1){1'b0}},1'b1}<<rx_count[6:0];
                    length_in_range<=rx_count>=63 && rx_count<128;
                    if(rx_count==254)rx_count_full<=1;
                    if(!rx_count_full)rx_count<=rx_count+1'b1;
                    if(rx_count>=128 || rx_byte_error)reject_frame<=1;
                    rx_crc<=crc_byte(rx_crc,rx_byte);
                end
            end else if(rx_active)begin
                rx_frames<=rx_frames+1'b1;last_length<=rx_count;
                if(receiving && !reject_frame && length_in_range && rx_crc==32'hdebb20e3)begin
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
    (* syn_preserve=1, syn_keep=1 *) reg clear_cache2_disp,lookup2_valid_disp;
    wire found2_disp=lookup2_valid_disp && !clear_cache2_disp;
    reg learn_pending,learn_match0,learn_match1;
    wire found=lookup_valid && !clear_cache;
    wire found2=lookup2_valid && !clear_cache2;
    wire found_comb=cache_valid0 && cache_ip0==target_ip;
    wire found2_comb=cache_valid1 && cache_ip1==target2_ip;
    // Compute from the SAME pre-edge cache inputs as lookup_valid/refresh_due.
    // These are local copies of the current-cycle predicate, not a delayed hit.
    wire invalidate_lookup0=clear_cache || (parse_state==P_COMMAND && (memory[47]==2 || memory[47]==4));
    wire invalidate_lookup1=clear_cache2 || (parse_state==P_COMMAND && (memory[47]==5 || memory[47]==8));
    // Endpoint IP also drives ARP/video/replies. Independent, same-edge local
    // copies keep that cross-module fanout out of the new cache predicate D path.
    (* syn_preserve=1, syn_keep=1 *) reg [31:0] retry_target_ip0,retry_target_ip1;
    wire retry_found0_comb=cache_valid0 && cache_ip0==retry_target_ip0;
    wire retry_found1_comb=cache_valid1 && cache_ip1==retry_target_ip1;
    wire fresh_cache0_comb=retry_found0_comb && cache_age0>(CACHE_SECONDS>5?5:1) && !invalidate_lookup0;
    wire fresh_cache1_comb=retry_found1_comb && cache_age1>(CACHE_SECONDS>5?5:1) && !invalidate_lookup1;
    (* syn_preserve=1, syn_keep=1 *) reg retry_hit_clear0,retry_hit_clear1;
    (* syn_preserve=1, syn_keep=1 *) reg retry_hit_clear0_disp,retry_hit_clear1_disp;
    (* syn_preserve=1, syn_keep=1 *) reg clear_cache_retry,clear_cache2_retry,clear_cache_disp;
    wire arp_needed0_disp=!retry_hit_clear0_disp || clear_cache_disp;
    wire arp_needed1_disp=!retry_hit_clear1_disp || clear_cache2_disp;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            retry_hit_clear0<=0;retry_hit_clear1<=0;
            retry_hit_clear0_disp<=0;retry_hit_clear1_disp<=0;
        end else begin
            retry_hit_clear0<=fresh_cache0_comb;retry_hit_clear1<=fresh_cache1_comb;
            retry_hit_clear0_disp<=fresh_cache0_comb;retry_hit_clear1_disp<=fresh_cache1_comb;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin lookup_valid<=0;lookup2_valid<=0;lookup2_valid_disp<=0;refresh_due<=0;refresh2_due<=0;found_mac<=0;found2_mac<=0;end
        else begin
            lookup_valid<=found_comb;lookup2_valid<=found2_comb;lookup2_valid_disp<=found2_comb;
            found_mac<=cache_mac0;found2_mac<=cache_mac1;
            refresh_due<=found_comb && cache_age0<=(CACHE_SECONDS>5?5:1);
            refresh2_due<=found2_comb && cache_age1<=(CACHE_SECONDS>5?5:1);
            if(clear_cache || (parse_state==P_COMMAND && (memory[47]==2 || memory[47]==4)))lookup_valid<=0;
            if(clear_cache2 || (parse_state==P_COMMAND && (memory[47]==5 || memory[47]==8)))begin lookup2_valid<=0;lookup2_valid_disp<=0;end
        end
    end
    assign cache_hit=found;assign target_mac=found_mac;
    assign cache2_hit=found2;assign target2_mac=found2_mac;
    localparam SECOND_BITS=CLK_HZ<2 ? 1 : $clog2(CLK_HZ);
    reg [SECOND_BITS-1:0] second_count;reg second_tick;
    // ARP uses millisecond retries. A 28-bit decrementer under the parser and
    // dispatch priority tree unnecessarily lengthens the 125MHz critical path.
    localparam RETRY_DIV=CLK_HZ>=1000 ? CLK_HZ/1000 : 1;
    localparam RETRY_SHORT=CLK_HZ>=1000 ? 500 : CLK_HZ/2;
    localparam RETRY_LONG=CLK_HZ>=1000 ? 2000 : CLK_HZ*2;
    localparam RETRY_BITS=$clog2(RETRY_LONG+1);
    localparam RETRY_DIV_BITS=RETRY_DIV<2 ? 1 : $clog2(RETRY_DIV);
    reg [RETRY_DIV_BITS-1:0] retry_divider;
    reg retry_tick;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin retry_divider<=0;retry_tick<=RETRY_DIV==1;end
        else begin
            retry_tick<=RETRY_DIV==1 || (!retry_tick && retry_divider==RETRY_DIV-2);
            if(retry_tick)retry_divider<=0;else retry_divider<=retry_divider+1'b1;
        end
    end
    reg [RETRY_BITS-1:0] retry_count,retry_count2;
    reg arp_due0,arp_due1;
    // Retry reductions must not sit in front of the full dispatch priority tree.
    // A registered request may be one clock old; dispatch rechecks availability.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin arp_due0<=0;arp_due1<=0;end
        else begin
            arp_due0<=enabled && arp_needed0_disp && retry_count==0;
            arp_due1<=enabled2 && arp_needed1_disp && retry_count2==0;
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
        P_VISION=7,P_VISION_WAIT=8,P_VISION_REPLY=9,P_IP_ADD=10,P_UDP_ADD=11,P_MOTOR=12;
    reg [3:0] parse_state;reg [6:0] scan_index;
    reg [15:0] scan_word;
    reg parsing_vision,parsing_motor,vision_pending;
    reg motor_shape;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)motor_shape<=0;
        else motor_shape<=ip_length==52 && udp_length==32 && frame_length==70 && {memory[36],memory[37]}==MOTOR_PORT;
    wire motor_datagram=MOTOR_ENABLE && motor_shape && !motor_valid;
    reg [255:0] vision_reply;
    reg [47:0] vision_mac;reg [31:0] vision_ip;reg [15:0] vision_peer_port;
    reg cmd_clear0,cmd_clear1;
    (* syn_preserve=1, syn_keep=1 *) reg cmd_set0,cmd_set1,cmd_set0_lookup,cmd_set1_lookup;
    reg control_shape,vision_shape,ip_envelope_ok,command_ok,requested_ok0,requested_ok1,vision_source_ok,video_length_ok;
    reg [5:0] vision_mac_equal;
    reg [3:0] vision_ip_equal;
    integer identity_index;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cmd_clear0<=0;cmd_clear1<=0;cmd_set0<=0;cmd_set1<=0;cmd_set0_lookup<=0;cmd_set1_lookup<=0;
            control_shape<=0;vision_shape<=0;ip_envelope_ok<=0;command_ok<=0;requested_ok0<=0;requested_ok1<=0;
            vision_mac_equal<=0;vision_ip_equal<=0;vision_source_ok<=0;video_length_ok<=0;
        end else begin
            // Mailbox and endpoint identity are stable throughout checksum scanning.
            // Register command acceptance before the P_COMMAND priority tree.
            cmd_clear0<=command_ok && ((memory[47]==2 && requested_ok0) || memory[47]==4);
            cmd_clear1<=command_ok && ((memory[47]==5 && requested_ok1) || memory[47]==8);
            cmd_set0<=command_ok && memory[47]==2 && requested_ok0;
            cmd_set1<=command_ok && memory[47]==5 && requested_ok1;
            cmd_set0_lookup<=command_ok && memory[47]==2 && requested_ok0;
            cmd_set1_lookup<=command_ok && memory[47]==5 && requested_ok1;
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
    wire [255:0] send_control_payload;
    reg event_pending;reg [255:0] event_buffer;
    assign event_ready=!event_pending;
    reg wait_first,wait_first2,video_pending,video_is_first;
    wire eligible0=enabled && found && (!wait_first || video_is_first);
    wire eligible1=enabled2 && found2 && (!wait_first2 || video_is_first);
    reg video_route0,video_route1;
    reg [47:0] video_route_mac;reg [31:0] video_route_ip;reg [15:0] video_route_port;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin video_route0<=0;video_route1<=0;video_route_mac<=0;video_route_ip<=0;video_route_port<=0;end
        else begin
            video_route0<=eligible0;video_route1<=eligible1;
            video_route_mac<=eligible0?found_mac:found2_mac;
            video_route_ip<=eligible0?target_ip:target2_ip;
            video_route_port<=eligible0?target_port:target2_port;
        end
    end
    reg replay_pending,using_replay;
    wire [47:0] replay_mac;wire [31:0] replay_ip;wire [15:0] replay_port;
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
    localparam S_IDLE=0,S_UDP=1,S_ARP=2,S_GAP=3,S_DISPATCH=4,S_ARP_LOAD=5;
    localparam A_NONE=0,A_REPLY=1,A_CONTROL=2,A_VISION=3,A_ARP0=4,A_ARP1=5,A_VIDEO=6,A_EVENT=7;
    localparam A_TELEMETRY=8;
    // When telemetry is enabled, rotate the three streaming classes. A camera
    // without fresh images may emit continuous unavailable-status snapshots;
    // those snapshots must not starve telemetry or pending video.
    reg [1:0] stream_turn;
    wire want_event=EVENT_ENABLE && event_pending && enabled2 && found2;
    wire telemetry_primary_ready=TELEMETRY_MIRROR_PRIMARY && enabled && found;
    wire telemetry_secondary_ready=enabled2 && found2;
    wire select_primary_telemetry=telemetry_primary_pending && telemetry_primary_ready;
    wire select_primary_telemetry_disp=telemetry_primary_pending && TELEMETRY_MIRROR_PRIMARY && enabled && found;
    wire want_telemetry=TELEMETRY_ENABLE && ((telemetry_pending && telemetry_secondary_ready) || select_primary_telemetry);
    reg [3:0] stream_action;
    always @*begin
        case(stream_turn)
            1:stream_action=want_telemetry?A_TELEMETRY:video_pending?A_VIDEO:want_event?A_EVENT:A_NONE;
            2:stream_action=video_pending?A_VIDEO:want_event?A_EVENT:want_telemetry?A_TELEMETRY:A_NONE;
            default:stream_action=want_event?A_EVENT:want_telemetry?A_TELEMETRY:video_pending?A_VIDEO:A_NONE;
        endcase
    end
    reg [3:0] action;

    wire [3:0] selected_action=TELEMETRY_ENABLE ?
        (arp_reply_pending ? A_REPLY : control_pending ? A_CONTROL : vision_pending ? A_VISION :
         arp_due0 ? A_ARP0 : arp_due1 ? A_ARP1 : stream_action) :
        (arp_reply_pending ? A_REPLY : control_pending ? A_CONTROL : vision_pending ? A_VISION :
         want_event ? A_EVENT : arp_due0 ? A_ARP0 : arp_due1 ? A_ARP1 : video_pending ? A_VIDEO : A_NONE);
    wire want_event_disp=EVENT_ENABLE && event_pending && enabled2 && found2_disp;
    wire want_telemetry_disp=TELEMETRY_ENABLE && ((telemetry_pending && enabled2 && found2_disp) || select_primary_telemetry_disp);
    reg [3:0] stream_action_disp;
    always @*begin
        case(stream_turn)
            1:stream_action_disp=want_telemetry_disp?A_TELEMETRY:video_pending?A_VIDEO:want_event_disp?A_EVENT:A_NONE;
            2:stream_action_disp=video_pending?A_VIDEO:want_event_disp?A_EVENT:want_telemetry_disp?A_TELEMETRY:A_NONE;
            default:stream_action_disp=want_event_disp?A_EVENT:want_telemetry_disp?A_TELEMETRY:video_pending?A_VIDEO:A_NONE;
        endcase
    end


    wire [3:0] selected_action_disp=TELEMETRY_ENABLE ?
        (arp_reply_pending ? A_REPLY : control_pending ? A_CONTROL : vision_pending ? A_VISION :
         arp_due0 ? A_ARP0 : arp_due1 ? A_ARP1 : stream_action_disp) :
        (arp_reply_pending ? A_REPLY : control_pending ? A_CONTROL : vision_pending ? A_VISION :
         want_event_disp ? A_EVENT : arp_due0 ? A_ARP0 : arp_due1 ? A_ARP1 : video_pending ? A_VIDEO : A_NONE);
    wire selected_udp=selected_action==A_CONTROL || selected_action==A_VISION ||
        selected_action==A_EVENT || selected_action==A_TELEMETRY || selected_action==A_VIDEO;


    localparam RETRY_LO=RETRY_BITS>6 ? 6 : RETRY_BITS-1;
    (* syn_preserve=1, syn_keep=1 *) reg retry_borrow,retry_borrow2;
    // borrow describes the CURRENT low segment, so the high subtractor sees
    // only one registered bit. It advances only on an actual decrement.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin retry_count<=0;retry_count2<=0;retry_borrow<=1;retry_borrow2<=1;end
        else begin
            if(retry_tick && retry_count!=0)begin
                retry_count[RETRY_LO-1:0]<=retry_count[RETRY_LO-1:0]-1'b1;
                retry_count[RETRY_BITS-1:RETRY_LO]<=retry_count[RETRY_BITS-1:RETRY_LO]-retry_borrow;
                retry_borrow<=retry_count[RETRY_LO-1:0]==1;
            end
            if(retry_tick && retry_count2!=0)begin
                retry_count2[RETRY_LO-1:0]<=retry_count2[RETRY_LO-1:0]-1'b1;
                retry_count2[RETRY_BITS-1:RETRY_LO]<=retry_count2[RETRY_BITS-1:RETRY_LO]-retry_borrow2;
                retry_borrow2<=retry_count2[RETRY_LO-1:0]==1;
            end
            if((retry_hit_clear0 && !clear_cache_retry) || (parse_state==P_COMMAND && cmd_clear0))begin retry_count<=0;retry_borrow<=1;end
            if((retry_hit_clear1 && !clear_cache2_retry) || (parse_state==P_COMMAND && cmd_clear1))begin retry_count2<=0;retry_borrow2<=1;end
            if(send_state==S_DISPATCH && action==A_ARP0 && arp_due0 && enabled && (!retry_hit_clear0 || clear_cache_retry))begin
                retry_count<=attempts<3 ? RETRY_SHORT : RETRY_LONG;
                retry_borrow<=attempts<3 ? (RETRY_SHORT % (1<<RETRY_LO))==0 : (RETRY_LONG % (1<<RETRY_LO))==0;
            end
            if(send_state==S_DISPATCH && action==A_ARP1 && arp_due1 && enabled2 && (!retry_hit_clear1 || clear_cache2_retry))begin
                retry_count2<=attempts2<3 ? RETRY_SHORT : RETRY_LONG;
                retry_borrow2<=attempts2<3 ? (RETRY_SHORT % (1<<RETRY_LO))==0 : (RETRY_LONG % (1<<RETRY_LO))==0;
            end
        end
    end

    reg telemetry_pending,telemetry_primary_pending,sending_telemetry;
    reg [511:0] telemetry_buffer,send_telemetry_payload;
    assign telemetry_ready=TELEMETRY_ENABLE && rst_n && (telemetry_secondary_ready || telemetry_primary_ready) && !telemetry_pending && !telemetry_primary_pending;
    reg udp_start,sending_video;wire [47:0] send_mac;wire [31:0] send_ip;
    wire [15:0] send_port,send_source_port,send_length;
    reg [5:0] control_index;
    wire udp_done,udp_read,udp_valid,udp_clock;wire [7:0] udp_data;
    wire [7:0] udp_payload=sending_video?(using_replay?replay_word:video_data):
        sending_telemetry ? send_telemetry_payload[511:504] : send_control_payload[255:248];
    eth_udp_tx_gmii udp(.clk125m(clk),.reset_p(network_reset),.tx_en_pulse(udp_start),.tx_done(udp_done),
        .dst_mac(send_mac),.src_mac(LOCAL_MAC),.dst_ip(send_ip),.src_ip(LOCAL_IP),
        .dst_port(send_port),.src_port(send_source_port),.data_length(send_length),
        .payload_req_o(udp_read),.payload_dat_i(udp_payload),.gmii_tx_clk(udp_clock),.gmii_txen(udp_valid),.gmii_txd(udp_data));
    assign video_read=udp_read && sending_video && !using_replay && send_state==S_UDP;

    // Complete minimum-sized Ethernet ARP frame: 8-byte preamble, 60-byte frame, 4-byte FCS.
    reg [6:0] arp_index;reg [31:0] arp_crc;reg arp_payload_active;
    reg [47:0] arp_send_mac,arp_target_mac;reg [31:0] arp_target_ip;reg [15:0] arp_opcode;
    reg [479:0] arp_body,arp_seed;
    (* syn_preserve=1, syn_keep=1 *) reg [14:0] arp_load_group,arp_shift_group;
    wire arp_issue=send_state==S_DISPATCH && (action==A_REPLY ||
        (action==A_ARP0 && arp_due0 && enabled && arp_needed0_disp) ||
        (action==A_ARP1 && arp_due1 && enabled2 && arp_needed1_disp));
    // Header selection has a whole cycle before locally registered load CEs.
    // The immutable seed freezes IP/MAC together, including endpoint changes.
    always @(posedge clk)
        arp_seed<={action==A_REPLY ? reply_mac : 48'hffffffffffff,LOCAL_MAC,
            16'h0806,16'h0001,16'h0800,8'd6,8'd4,16'd0,LOCAL_MAC,LOCAL_IP,
            action==A_REPLY ? reply_mac : 48'd0,
            action==A_REPLY ? reply_ip : action==A_ARP0 ? target_ip : target2_ip,144'd0};
    genvar arp_group;
    generate for(arp_group=0;arp_group<15;arp_group=arp_group+1)begin:arp_payload_group
        always @(posedge clk or negedge rst_n)
            if(!rst_n)begin arp_load_group[arp_group]<=0;arp_shift_group[arp_group]<=0;end
            else begin
                arp_load_group[arp_group]<=arp_issue;
                arp_shift_group[arp_group]<=send_state==S_ARP && arp_index>=7 && arp_index<67;
            end
        wire [479:0] seed_opcode={arp_seed[479:320],arp_opcode,arp_seed[303:0]};
        if(arp_group==0)begin
            always @(posedge clk)
                if(arp_load_group[arp_group])arp_body[31:0]<=seed_opcode[31:0];
                else if(arp_shift_group[arp_group])arp_body[31:0]<={arp_body[23:0],8'd0};
        end else begin
            always @(posedge clk)
                if(arp_load_group[arp_group])arp_body[arp_group*32+:32]<=seed_opcode[arp_group*32+:32];
                else if(arp_shift_group[arp_group])arp_body[arp_group*32+:32]<=arp_body[arp_group*32-8+:32];
        end
    end endgenerate
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
            motor_valid<=0;motor_frame<=0;parsing_motor<=0;
            req_sync0<=0;req_sync1<=0;ack_toggle<=0;parse_state<=P_IDLE;check_sum<=0;
            enabled<=DEFAULT_IP!=0;
            enabled2<=0;wait_first2<=1;
            replay_pending<=0;using_replay<=0;control_second<=0;
            learn<=0;learned_ip<=0;learned_mac<=0;clear_cache<=0;clear_cache2<=0;clear_cache2_disp<=0;clear_cache_retry<=0;clear_cache2_retry<=0;clear_cache_disp<=0;
            reply_mac<=0;reply_ip<=0;arp_reply_pending<=0;control_pending<=0;ack_build<=0;
            control_mac<=0;control_ip<=0;control_id<=0;control_port<=0;control_opcode<=0;control_error<=0;control_payload<=0;arp_debug<=0;
            send_state<=S_IDLE;udp_start<=0;sending_video<=0;
            control_index<=0;video_done<=0;video_pending<=0;video_is_first<=0;wait_first<=1;
            attempts<=0;attempts2<=0;arp_requests<=0;rx_packets<=0;arp_index<=0;arp_crc<=32'hffffffff;
            arp_send_mac<=0;arp_target_mac<=0;arp_target_ip<=0;arp_opcode<=0;gap<=0;
            parsing_vision<=0;vision_valid<=0;vision_payload<=0;vision_pending<=0;
            vision_reply<=0;vision_mac<=0;vision_ip<=0;vision_peer_port<=0;
            scan_word<=0;action<=A_NONE;event_pending<=0;event_buffer<=0;arp_payload_active<=0;
            telemetry_pending<=0;telemetry_primary_pending<=0;sending_telemetry<=0;telemetry_buffer<=0;stream_turn<=0;
        end else begin
            req_sync0<=request_toggle;req_sync1<=req_sync0;learn<=0;clear_cache<=0;clear_cache2<=0;clear_cache2_disp<=0;clear_cache_retry<=0;clear_cache2_retry<=0;clear_cache_disp<=0;
            udp_start<=0;video_done<=0;vision_valid<=0;
            if(motor_valid && motor_ready)motor_valid<=0;
            if(!enabled2 || !found2)event_pending<=0;
            if(EVENT_ENABLE && event_valid && event_ready && enabled2 && found2)begin event_buffer<=event_payload;event_pending<=1;end
            // Do not acknowledge until this complete datagram has a queue slot.
            // Snapshot data independently of wide admission/ARP conditions.
            // The CDC producer holds it until acceptance; pending then locks it.
            if(TELEMETRY_ENABLE && !telemetry_pending && !telemetry_primary_pending)telemetry_buffer<=telemetry_payload;
            if(TELEMETRY_MIRROR_PRIMARY && !telemetry_primary_ready)telemetry_primary_pending<=0;
            if(TELEMETRY_MIRROR_PRIMARY && !telemetry_secondary_ready)telemetry_pending<=0;
            if(TELEMETRY_ENABLE && telemetry_valid && telemetry_ready)begin
                telemetry_pending<=telemetry_secondary_ready;
                telemetry_primary_pending<=telemetry_primary_ready;
            end
            if(found && !refresh_due)begin attempts<=0;end
            if(found2 && !refresh2_due)begin attempts2<=0;end
            if(video_start)begin video_pending<=1;video_is_first<=video_first;end
            if(udp_read && !sending_video)control_index<=control_index+1'b1;
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
                        (control_datagram || vision_datagram || motor_datagram))begin
                        parsing_motor<=motor_datagram;parsing_vision<=vision_datagram;check_sum<=0;parse_state<=P_IP;
                    end else begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                end
                P_IP:if(scan_index<34)begin
                    scan_word<={memory[scan_index],memory[scan_index+7'd1]};parse_state<=P_IP_ADD;
                end else parse_state<=P_IP_WAIT;
                P_IP_ADD:begin check_sum<=check_sum+scan_word;parse_state<=P_IP;end
                P_IP_WAIT:if(!checksum_ok)begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                else if({memory[40],memory[41]}==0)parse_state<=parsing_motor ? P_MOTOR : parsing_vision ? P_VISION : P_COMMAND;
                else begin
                    check_sum<=peer_ip[31:16]+{16'd0,peer_ip[15:0]}+{16'd0,LOCAL_IP[31:16]}+{16'd0,LOCAL_IP[15:0]}+32'd17+{16'd0,udp_length};
                    parse_state<=P_UDP;
                end
                P_UDP:if(scan_index<(parsing_motor ? 66 : parsing_vision ? 74 : 62))begin
                    scan_word<={memory[scan_index],memory[scan_index+7'd1]};parse_state<=P_UDP_ADD;
                end else parse_state<=P_UDP_WAIT;
                P_UDP_ADD:begin check_sum<=check_sum+scan_word;parse_state<=P_UDP;end
                P_UDP_WAIT:if(checksum_ok)parse_state<=parsing_motor ? P_MOTOR : parsing_vision ? P_VISION : P_COMMAND;
                else begin ack_toggle<=req_sync1;parse_state<=P_IDLE;end
                P_MOTOR:begin
                    // Complete datagram only, after Ethernet/IP/UDP validation.
                    // Inner V3 CRC and opcode/sequence guards run in chassis.
                    for(v=0;v<24;v=v+1)motor_frame[v*8 +: 8]<=memory[42+v];
                    motor_valid<=1;ack_toggle<=req_sync1;parse_state<=P_IDLE;
                end
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
                                enabled<=1;wait_first<=1;attempts<=0;clear_cache<=1;clear_cache_retry<=1;clear_cache_disp<=1;
                            end else control_error<=1;
                            3:begin enabled<=0;wait_first<=1;end
                            4:begin clear_cache<=1;clear_cache_retry<=1;clear_cache_disp<=1;wait_first<=1;attempts<=0;end
                            5:if(requested_ok1)begin
                                enabled2<=1;wait_first2<=1;attempts2<=0;clear_cache2<=1;clear_cache2_retry<=1;clear_cache2_disp<=1;
                            end else control_error<=1;
                            6:begin enabled2<=0;wait_first2<=1;end
                            7:begin end
                            8:begin clear_cache2<=1;clear_cache2_retry<=1;clear_cache2_disp<=1;wait_first2<=1;attempts2<=0;end
                            default:control_error<=2;
                        endcase
                    end
                end
            endcase
            case(send_state)
                S_IDLE:begin
                    action<=selected_action;
                    send_state<=S_DISPATCH;
                end
                S_DISPATCH:begin
                    send_state<=S_IDLE;
                    sending_telemetry<=0;
                    arp_payload_active<=0;
                    case(action)
                    A_TELEMETRY:begin
                        stream_turn<=2;
                        if(select_primary_telemetry)telemetry_primary_pending<=0;else telemetry_pending<=0;
                        sending_telemetry<=1;sending_video<=0;


                        if(select_primary_telemetry || (telemetry_pending && telemetry_secondary_ready))begin udp_start<=1;send_state<=S_UDP;end
                    end
                    A_REPLY:begin
                        arp_send_mac<=reply_mac;arp_target_mac<=reply_mac;arp_target_ip<=reply_ip;arp_opcode<=2;
                        arp_reply_pending<=0;arp_index<=0;arp_crc<=32'hffffffff;send_state<=S_ARP_LOAD;
                    end
                    A_CONTROL:begin

                        sending_video<=0;control_index<=0;control_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end
                    A_VISION:begin

                        sending_video<=0;control_index<=0;vision_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end
                    A_EVENT:begin
                        stream_turn<=1;
                        event_pending<=0;
                        // Snapshot data unconditionally; only a valid destination
                        // permits udp_start. This removes wide destination enables.

                        sending_video<=0;control_index<=0;
                        if(enabled2 && found2)begin
                            udp_start<=1;send_state<=S_UDP;
                        end
                    end
                    A_ARP0:begin
                        arp_send_mac<=48'hffffffffffff;arp_target_mac<=0;arp_target_ip<=target_ip;arp_opcode<=1;
                        arp_index<=0;arp_crc<=32'hffffffff;
                        if(arp_due0 && enabled && arp_needed0_disp)begin
                            send_state<=S_ARP_LOAD;arp_requests<=arp_requests+1'b1;
                            if(attempts<3)attempts<=attempts+1'b1;
                        end
                    end
                    A_ARP1:begin
                        arp_send_mac<=48'hffffffffffff;arp_target_mac<=0;arp_target_ip<=target2_ip;arp_opcode<=1;
                        arp_index<=0;arp_crc<=32'hffffffff;
                        if(arp_due1 && enabled2 && arp_needed1_disp)begin
                            send_state<=S_ARP_LOAD;arp_requests<=arp_requests+1'b1;
                            if(attempts2<3)attempts2<=attempts2+1'b1;
                        end
                    end
                    A_VIDEO:begin
                        stream_turn<=0;
                        video_pending<=0;
                        // Route selection is a separate stage from dispatch.
                        // If admission changed since that stage, drop this
                        // source packet rather than mixing endpoint identities.



                        sending_video<=1;
                        if(!found)wait_first<=1;
                        if(!found2)wait_first2<=1;
                        if((!eligible0 && !eligible1) || !video_length_ok ||
                                video_route0!=eligible0 || video_route1!=eligible1)begin video_done<=1;end
                        else begin
                            if(eligible0)wait_first<=0;
                            if(eligible1)wait_first2<=0;
                            replay_pending<=eligible0 && eligible1;using_replay<=0;
                            udp_start<=1;send_state<=S_UDP;
                        end
                    end
                    default:begin end
                    endcase
                end
                S_UDP:if(udp_done)begin
                    if(sending_video && !replay_pending)video_done<=1;send_state<=S_GAP;gap<=0;
                end
                S_ARP_LOAD:send_state<=S_ARP;
                S_ARP:begin
                    arp_payload_active<=arp_index>=7 && arp_index<67;
                    if(arp_payload_active)arp_crc<=crc_byte(arp_crc,arp_body[479:472]);
                    if(arp_index==71)begin send_state<=S_GAP;gap<=0;end else arp_index<=arp_index+1'b1;
                end
                S_GAP:if(gap==15)begin
                    if(replay_pending)begin

                        using_replay<=1;replay_pending<=0;udp_start<=1;send_state<=S_UDP;
                    end else send_state<=S_IDLE;
                end else gap<=gap+1'b1;
            endcase
        end
    end
    // Mailbox/acceptance flags have settled for the complete checksum scan.
    // Predecode SET commands alongside cmd_clear; update endpoints on the SAME
    // P_COMMAND edge, outside the unrelated reply/parser priority tree.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            target_ip<=DEFAULT_IP;target_port<=DEFAULT_PORT;
            target2_ip<=0;target2_port<=DEFAULT_PORT;
            retry_target_ip0<=DEFAULT_IP;retry_target_ip1<=0;
        end else begin
            if(parse_state==P_COMMAND && cmd_set0)begin target_ip<=requested_ip;target_port<=requested_port;end
            if(parse_state==P_COMMAND && cmd_set1)begin target2_ip<=requested_ip;target2_port<=requested_port;end
            if(parse_state==P_COMMAND && cmd_set0_lookup)retry_target_ip0<=requested_ip;
            if(parse_state==P_COMMAND && cmd_set1_lookup)retry_target_ip1<=requested_ip;
        end
    end
    // Index writes do not depend on envelope admission or checksum verdicts.
    // Invalid envelopes never use it; this removes that long enable path.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)scan_index<=0;
        else case(parse_state)
            P_CLASSIFY:scan_index<=14;
            P_IP_WAIT:scan_index<=34;
            P_IP_ADD,P_UDP_ADD:scan_index<=scan_index+7'd2;
            default:begin end
        endcase
    end


    // Every <=32-bit destination word has local registered operation/load
    // controls. Snapshot timing remains SIDLE -> SDISPATCH, with no added
    // consumer-visible handshake latency.
    localparam [8:0] CONTROL_OPS=(9'b1<<A_CONTROL)|(9'b1<<A_VISION)|(9'b1<<A_EVENT);
    localparam [8:0] UDP_OPS=CONTROL_OPS|(9'b1<<A_TELEMETRY)|(9'b1<<A_VIDEO);
    genvar word_index,operation_index;
    generate for(word_index=0;word_index<8;word_index=word_index+1)begin:control_words
        wire [287:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*32+:32]=operation_index==A_CONTROL ? control_payload[word_index*32+:32] :
                operation_index==A_VISION ? vision_reply[word_index*32+:32] :
                operation_index==A_EVENT ? event_buffer[word_index*32+:32] : 32'd0;
        end
        wire [31:0] shifted;
        if(word_index==0)assign shifted={send_control_payload[23:0],8'd0};
        else assign shifted=send_control_payload[word_index*32-8+:32];
        network_dispatch_word #(.WIDTH(32),.OPERATIONS(CONTROL_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),.replay_value(32'd0),
            .shift(udp_read && !sending_video && !sending_telemetry),.shift_value(shifted),
            .value(send_control_payload[word_index*32+:32]));
    end endgenerate
    generate for(word_index=0;word_index<16;word_index=word_index+1)begin:telemetry_words
        (* syn_preserve=1, syn_keep=1 *) reg load_word;
        always @(posedge clk or negedge rst_n)
            if(!rst_n)load_word<=0;else load_word<=send_state==S_IDLE;
        wire [31:0] shifted;
        if(word_index==0)assign shifted={send_telemetry_payload[23:0],8'd0};
        else assign shifted=send_telemetry_payload[word_index*32-8+:32];
        always @(posedge clk or negedge rst_n)
            if(!rst_n)send_telemetry_payload[word_index*32+:32]<=0;
            else if(load_word)send_telemetry_payload[word_index*32+:32]<=telemetry_buffer[word_index*32+:32];
            else if(udp_read)send_telemetry_payload[word_index*32+:32]<=shifted;
    end endgenerate

    wire [431:0] send_mac_choices={(select_primary_telemetry_disp ? found_mac : found2_mac),found2_mac,video_route_mac,48'd0,48'd0,vision_mac,control_mac,48'd0,48'd0};
    generate for(word_index=0;word_index<(48+31)/32;word_index=word_index+1)begin:send_mac_words
        localparam WIDTH=(48-word_index*32)<32 ? 48-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=send_mac_choices[operation_index*48+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS(UDP_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(send_state==S_GAP && gap==14 && replay_pending),
            .replay_value(replay_mac[word_index*32+:WIDTH]),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(send_mac[word_index*32+:WIDTH]));
    end endgenerate

    wire [287:0] send_ip_choices={(select_primary_telemetry_disp ? target_ip : target2_ip),target2_ip,video_route_ip,32'd0,32'd0,vision_ip,control_ip,32'd0,32'd0};
    generate for(word_index=0;word_index<(32+31)/32;word_index=word_index+1)begin:send_ip_words
        localparam WIDTH=(32-word_index*32)<32 ? 32-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=send_ip_choices[operation_index*32+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS(UDP_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(send_state==S_GAP && gap==14 && replay_pending),
            .replay_value(replay_ip[word_index*32+:WIDTH]),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(send_ip[word_index*32+:WIDTH]));
    end endgenerate

    wire [143:0] send_port_choices={TELEMETRY_PORT,EVENT_PORT,video_route_port,16'd0,16'd0,vision_peer_port,control_port,16'd0,16'd0};
    generate for(word_index=0;word_index<(16+31)/32;word_index=word_index+1)begin:send_port_words
        localparam WIDTH=(16-word_index*32)<32 ? 16-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=send_port_choices[operation_index*16+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS(UDP_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(send_state==S_GAP && gap==14 && replay_pending),
            .replay_value(replay_port[word_index*32+:WIDTH]),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(send_port[word_index*32+:WIDTH]));
    end endgenerate

    wire [143:0] send_source_port_choices={TELEMETRY_PORT,EVENT_PORT,VIDEO_SOURCE_PORT,16'd0,16'd0,VISION_PORT,CONTROL_PORT,16'd0,16'd0};
    generate for(word_index=0;word_index<(16+31)/32;word_index=word_index+1)begin:send_source_port_words
        localparam WIDTH=(16-word_index*32)<32 ? 16-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=send_source_port_choices[operation_index*16+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS(UDP_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),
            .replay_value({WIDTH{1'b0}}),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(send_source_port[word_index*32+:WIDTH]));
    end endgenerate

    wire [143:0] send_length_choices={16'd64,16'd32,video_length,16'd0,16'd0,16'd32,16'd32,16'd0,16'd0};
    generate for(word_index=0;word_index<(16+31)/32;word_index=word_index+1)begin:send_length_words
        localparam WIDTH=(16-word_index*32)<32 ? 16-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=send_length_choices[operation_index*16+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS(UDP_OPS)) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),
            .replay_value({WIDTH{1'b0}}),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(send_length[word_index*32+:WIDTH]));
    end endgenerate

    wire [431:0] replay_mac_choices={48'd0,48'd0,found2_mac,48'd0,48'd0,48'd0,48'd0,48'd0,48'd0};
    generate for(word_index=0;word_index<(48+31)/32;word_index=word_index+1)begin:replay_mac_words
        localparam WIDTH=(48-word_index*32)<32 ? 48-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=replay_mac_choices[operation_index*48+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS((9'b1<<A_VIDEO))) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),
            .replay_value({WIDTH{1'b0}}),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(replay_mac[word_index*32+:WIDTH]));
    end endgenerate

    wire [287:0] replay_ip_choices={32'd0,32'd0,target2_ip,32'd0,32'd0,32'd0,32'd0,32'd0,32'd0};
    generate for(word_index=0;word_index<(32+31)/32;word_index=word_index+1)begin:replay_ip_words
        localparam WIDTH=(32-word_index*32)<32 ? 32-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=replay_ip_choices[operation_index*32+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS((9'b1<<A_VIDEO))) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),
            .replay_value({WIDTH{1'b0}}),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(replay_ip[word_index*32+:WIDTH]));
    end endgenerate

    wire [143:0] replay_port_choices={16'd0,16'd0,target2_port,16'd0,16'd0,16'd0,16'd0,16'd0,16'd0};
    generate for(word_index=0;word_index<(16+31)/32;word_index=word_index+1)begin:replay_port_words
        localparam WIDTH=(16-word_index*32)<32 ? 16-word_index*32 : 32;
        wire [9*WIDTH-1:0] choices;
        for(operation_index=0;operation_index<9;operation_index=operation_index+1)begin:choice
            assign choices[operation_index*WIDTH+:WIDTH]=replay_port_choices[operation_index*16+word_index*32+:WIDTH];
        end
        network_dispatch_word #(.WIDTH(WIDTH),.OPERATIONS((9'b1<<A_VIDEO))) word_register(
            .clk(clk),.rst_n(rst_n),.choose(send_state==S_IDLE),.selected_action(selected_action_disp),.choices(choices),
            .replay_predict(1'b0),
            .replay_value({WIDTH{1'b0}}),
            .shift(1'b0),.shift_value({WIDTH{1'b0}}),.value(replay_port[word_index*32+:WIDTH]));
    end endgenerate

endmodule

// A local dispatch control register for one data word. The one-hot selector is
// replicated per word; a global action bit never drives an entire wide bus.
module network_dispatch_word #(parameter WIDTH=32,parameter [8:0] OPERATIONS=0)(
    input clk,rst_n,choose,input [3:0] selected_action,input [9*WIDTH-1:0] choices,
    input replay_predict,input [WIDTH-1:0] replay_value,input shift,input [WIDTH-1:0] shift_value,
    output reg [WIDTH-1:0] value);
    (* syn_preserve=1, syn_keep=1 *) reg [8:0] operation;
    (* syn_preserve=1, syn_keep=1 *) reg load_word;
    reg replay_word;
    reg [WIDTH-1:0] selected_word;integer k;
    always @*begin
        selected_word=0;
        for(k=0;k<9;k=k+1)selected_word=selected_word | ({WIDTH{operation[k]}} & choices[k*WIDTH+:WIDTH]);
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin operation<=0;load_word<=0;replay_word<=0;end
        else begin
            if(choose)operation<=(9'b1<<selected_action)&OPERATIONS;
            load_word<=choose && OPERATIONS[selected_action];
            replay_word<=replay_predict;
        end
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n)value<=0;
        else if(load_word)value<=selected_word;
        else if(replay_word)value<=replay_value;
        else if(shift)value<=shift_value;
endmodule
