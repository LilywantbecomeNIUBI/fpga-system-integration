`timescale 1ns/1ps
// Reference modules are generated from the pre-change frozen source, not from
// the new segmented-counter implementation. Compare the fast safety plane
// every clock, including 8/16-bit carry boundaries and long expiry holds.
module tb_bridge_timing_equivalence;
    reg clk=0,rst=0,ready=1,flow_v=0,obs_v=0,sem_update=0,sem_valid=1;
    reg sink=0;reg [255:0] flow_data=0,obs_data=0,sem_data=0;
    reg result_valid=0;
    wire [255:0] result_payload={32'h56444554,8'd1,8'd1,16'd1,16'd1280,16'd720,16'd39,16'd900,
                                16'd256,16'd192,16'd768,16'd576,16'd0,48'd0};
    always #5 clk=~clk;
    wire s0,s1;wire signed [7:0] turn0,turn1;wire [7:0] speed0,speed1;
    wire [2:0] state0,state1;wire [3:0] led0,led1;
    wire fv,vv;wire [255:0] fp,vp;
    flow_event_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) flow(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.observation_valid(flow_v),.observation_payload(flow_data),
        .event_ready(sink),.event_valid(fv),.event_payload(fp),.stop_request(s0),.steer(turn0),.speed_limit(speed0),.control_state(state0));
    flow_event_bridge_reference #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) flow_ref(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.observation_valid(flow_v),.observation_payload(flow_data),
        .event_ready(sink),.event_valid(),.event_payload(),.stop_request(s1),.steer(turn1),.speed_limit(speed1),.control_state(state1));
    vision_v23_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) vision(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.observation_valid(obs_v),.observation_payload(obs_data),
        .semantic_update(sem_update),.semantic_valid(sem_valid),.semantic_payload(sem_data),
        .event_ready(sink),.event_valid(vv),.event_payload(vp),.led(led0));
    vision_v23_bridge_reference #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) vision_ref(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.observation_valid(obs_v),.observation_payload(obs_data),
        .semantic_update(sem_update),.semantic_valid(sem_valid),.semantic_payload(sem_data),
        .event_ready(sink),.event_valid(),.event_payload(),.led(led1));
    vision_result_core #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) core(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.frame_tick(obs_v),.frame_id(16'd1),
        .result_valid(result_valid),.result_payload(result_payload));
    vision_result_core_reference #(.CLK_HZ(1000),.TIMEOUT_MS(70000)) core_ref(
        .clk(clk),.rst_n(rst),.stream_ready(ready),.frame_tick(obs_v),.frame_id(16'd1),
        .result_valid(result_valid),.result_payload(result_payload));
    integer cycle=0,seed=32'h5102f;reg [255:0] held_f,held_v;reg stalled_f=0,stalled_v=0;
    always @(posedge clk)begin
        if(rst)begin
            if(stalled_f && (!fv || fp!==held_f))$fatal(1,"flow changed under stall");
            if(stalled_v && (!vv || vp!==held_v))$fatal(1,"vision changed under stall");
            stalled_f=fv && !sink;held_f=fp;stalled_v=vv && !sink;held_v=vp;
            #1;
            if({s0,turn0,speed0,state0,led0}!=={s1,turn1,speed1,state1,led1})$fatal(1,"fast safety mismatch cycle=%0d",cycle);
            if(flow.age!==flow_ref.age || vision.age!==vision_ref.age ||
               core.idle_cycles!=core_ref.idle_cycles || core.frame_idle_cycles!=core_ref.frame_idle_cycles ||
               {core.idle_due,core.frame_due,core.frame_expired}!=={core_ref.idle_due,core_ref.frame_due,core_ref.frame_expired})
                $fatal(1,"counter/deadline mismatch cycle=%0d",cycle);
            if({core.target_valid,core.accepted_pulse,core.reason,core.accepted,core.rejected}!==
               {core_ref.target_valid,core_ref.accepted_pulse,core_ref.reason,core_ref.accepted,core_ref.rejected})
                $fatal(1,"semantic accept/refresh mismatch");
            if((fv && (^fp)===1'bx) || (vv && (^vp)===1'bx))$fatal(1,"uninitialized queued event");
        end
    end
    initial begin
        repeat(4)@(negedge clk);rst=1;
        flow_data[255:224]=32'h4f465354;flow_data[201]=1;flow_data[79:72]=200;
        obs_data[255:216]={32'h46535441,8'd2};obs_data[208]=1;obs_data[199:192]=2;
        flow_v=1;obs_v=1;sem_update=1;
        for(cycle=0;cycle<85000;cycle=cycle+1)begin
            @(negedge clk);flow_v=0;obs_v=0;sem_update=0;
            if(cycle>=75000)begin
                flow_v=($random(seed)&7)==0;obs_v=($random(seed)&7)==0;
                sem_update=($random(seed)&15)==0;
                ready=($random(seed)&31)!=0;sem_valid=($random(seed)&31)!=0;
                flow_data[200]=$random(seed);flow_data[71:64]=$random(seed);
                flow_data[194:192]=$random(seed);obs_data[199:192]=$random(seed)&3;
                if(sem_update)sem_data[207:192]=$random(seed)&15;
                obs_data[175:160]=cycle%3==0 ? sem_data[207:192] : $random(seed)&15;
            end
            result_valid=cycle==75500 || cycle==76524 || cycle==80000;
            if(result_valid)begin obs_v=1;ready=1;end
            sink=cycle%19<11;
        end
        $display("PASS 85000 clocks: fast controls, LED, byte/word carry, expiry/refresh match frozen reference; both queues immutable");$finish;
    end
endmodule
