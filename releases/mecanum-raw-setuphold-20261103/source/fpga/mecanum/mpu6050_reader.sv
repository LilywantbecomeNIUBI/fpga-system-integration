// MPU6050 (WHO=68) / MPU6500 (WHO=70), AD0=0 by default.
// Both: +/-250 dps = 131 LSB/(deg/s); GYRO_CONFIG=0.
// MPU6500 FCHOICE_B=00, DLPF_CFG=3: 41Hz bandwidth, internal 1kHz.
// SMPLRT_DIV=9 selects 100Hz. MPU6050 uses the same register values
// (its DLPF=3 gyro bandwidth is 42Hz). No unknown identities accepted.
// Register configuration is read back; each gyro burst requires DATA_RDY.
module mc_mpu6050_reader #(
 parameter CLK_HZ=50000000,I2C_HZ=100000,ADDRESS=7'h68,
 BOOT_US=100000,RESET_US=100000,SETTLE_US=50000,POLL_US=1000
)(
 input wire clk,reset_n,restart,inout wire scl,sda,
 output reg ready,fault,sample_valid,
 output reg signed [15:0] gx,gy,gz,
 output reg [31:0] sample_us,output reg [31:0] now_us,
 output reg [7:0] who_am_i,output reg [3:0] error_code,
 output wire stale,output reg [3:0] last_bus_error,output wire [1:0] bus_lines,output reg [15:0] bus_error_count
);
 localparam US_DIV=CLK_HZ/1000000;
 integer us_div,delay_count;reg [4:0] state;reg [2:0] step;
 reg [31:0] acquisition_start_us;
 reg req,reading;reg [7:0] address,value;reg [5:0] length;
 wire busy,done;wire [3:0] bus_error;wire [255:0] result;
 reg [2:0] restart_hold;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)restart_hold<=0;
  else if(restart)restart_hold<=7;
  else if(restart_hold!=0)restart_hold<=restart_hold-1'b1;
 end
 wire acquisition_reset_n=reset_n && !restart && restart_hold==0;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin last_bus_error<=0;bus_error_count<=0;end
  else if(done && bus_error!=0)begin
   last_bus_error<=bus_error;
   if(bus_error_count!=16'hffff)bus_error_count<=bus_error_count+1'b1;
  end
 end
 assign stale=!ready || fault || now_us-sample_us>25000;
 mc_imu_fast_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.TIMEOUT_US(5000)) bus(
  .clk(clk),.rst_n(acquisition_reset_n),.req(req),.read_not_write(reading),.device_addr(ADDRESS),
  .reg_addr(address),.write_data(value),.read_length(length),.busy(busy),.done(done),
  .error(bus_error),.read_data(result),.scl(scl),.sda(sda),.line_levels(bus_lines));
 function automatic [7:0] cfg_addr(input [2:0] n);
  case(n)0:cfg_addr=8'h6b;1:cfg_addr=8'h1a;2:cfg_addr=8'h19;3:cfg_addr=8'h1b;4:cfg_addr=8'h6c;default:cfg_addr=8'h38;endcase
 endfunction
 function automatic [7:0] cfg_value(input [2:0] n);
  case(n)0:cfg_value=1;1:cfg_value=3;2:cfg_value=9;5:cfg_value=1;default:cfg_value=0;endcase
 endfunction
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin now_us<=0;us_div<=0;end
  else if(us_div==US_DIV-1)begin us_div<=0;now_us<=now_us+1'b1;end
  else us_div<=us_div+1;
 end
 always @(posedge clk or negedge acquisition_reset_n)begin
  if(!acquisition_reset_n)begin
   state<=0;step<=0;req<=0;reading<=0;address<=0;value<=0;length<=1;delay_count<=0;
   ready<=0;fault<=0;sample_valid<=0;gx<=0;gy<=0;gz<=0;sample_us<=0;who_am_i<=0;error_code<=0;acquisition_start_us<=0;
  end else begin
   req<=0;sample_valid<=0;
   // Runtime failures retain the previous sample AND its original timestamp.
   // Keep polling; only a successful complete burst publishes a fresh sample.
   if(ready && (sample_us!=0 ? now_us-sample_us>25000 : now_us-acquisition_start_us>25000))fault<=1;
   case(state)
    0:if(delay_count>=US_DIV*BOOT_US)begin delay_count<=0;state<=1;end else delay_count<=delay_count+1;
    1:if(!busy)begin req<=1;reading<=1;address<=8'h75;length<=1;state<=2;end
    2:if(done)begin
     who_am_i<=result[7:0];
     if(bus_error!=0 || (result[7:0]!=8'h68 && result[7:0]!=8'h70))begin fault<=1;error_code<=1;state<=31;end
     else state<=3;
    end
    3:if(!busy)begin req<=1;reading<=0;address<=8'h6b;value<=8'h80;state<=4;end
    4:if(done)begin
     if(bus_error!=0)begin fault<=1;error_code<=2;state<=31;end else begin delay_count<=0;state<=5;end
    end
    5:if(delay_count>=US_DIV*RESET_US)begin step<=0;state<=6;end else delay_count<=delay_count+1;
    6:if(!busy)begin req<=1;reading<=0;address<=cfg_addr(step);value<=cfg_value(step);state<=7;end
    7:if(done)begin if(bus_error!=0)begin fault<=1;error_code<=3;state<=31;end else state<=8;end
    8:if(!busy)begin req<=1;reading<=1;address<=cfg_addr(step);length<=1;state<=9;end
    9:if(done)begin
     if(bus_error!=0 || result[7:0]!=cfg_value(step))begin fault<=1;error_code<=4;state<=31;end
     else if(step==5)begin delay_count<=0;state<=10;end else begin step<=step+1'b1;state<=6;end
    end
    10:if(delay_count>=US_DIV*SETTLE_US)begin ready<=1;acquisition_start_us<=now_us;delay_count<=0;state<=11;end else delay_count<=delay_count+1;
    11:if(delay_count>=US_DIV*POLL_US)begin delay_count<=0;state<=12;end else delay_count<=delay_count+1;
    12:if(!busy)begin req<=1;reading<=1;address<=8'h3a;length<=1;state<=13;end
    13:if(done)begin
     if(bus_error!=0)begin error_code<=5;delay_count<=0;state<=11;end
     else if(result[0])state<=14;else state<=11;
    end
    14:if(!busy)begin req<=1;reading<=1;address<=8'h43;length<=6;state<=15;end
    15:if(done)begin
     if(bus_error!=0)begin error_code<=6;delay_count<=0;state<=11;end
     else begin gx<=result[47:32];gy<=result[31:16];gz<=result[15:0];sample_us<=now_us;sample_valid<=1;fault<=0;error_code<=0;state<=11;end
    end
    31:begin
     ready<=0;fault<=1;
     // Before the first usable configuration, retry initialization indefinitely.
     if(delay_count>=US_DIV*BOOT_US)begin delay_count<=0;state<=0;end
     else delay_count<=delay_count+1;
    end
    default:begin fault<=1;ready<=0;error_code<=7;state<=31;end
   endcase
  end
 end
endmodule
