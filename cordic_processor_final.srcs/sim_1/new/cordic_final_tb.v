`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.02.2024 11:05:52
// Design Name: 
// Module Name: cordic_final_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
/*
module cordic_tb;

parameter PERIOD = 10; // Clock period in ns

// Inputs
reg clock;
reg signed [31:0] angle;
reg signed [15:0] Xin, Yin; // Adjust data width if needed

// Outputs
wire signed [16:0] Xout, Yout;


cordic_final cf(clock,angle,Xin,Yin,Xout,Yout);
// Reference values
real expected_sin, expected_cos;

// File I/O for error logging
integer err_file;

initial begin
    // Open error log file
    err_file = $fopen("cordic_errors.txt");
    $fwrite(err_file, "Angle,Expected Sin,Expected Cos,Actual Sin,Actual Cos,Error\n");
end

initial begin
    clock = 1'b0;
    #5;
    clock = 1'b1;
    #5;
    clock = 1'b0;
    forever #PERIOD begin
        clock = ~clock;
    end
end

initial begin
    // Sweep through all angles from 0 to 90 degrees in 1-degree increments
    for (angle = 0; angle <= 90; angle = angle + 1) begin
        // Calculate expected sine and cosine using a reference library function
        expected_sin = $sin(angle * $pi / 180);
        expected_cos = $cos(angle * $pi / 180);

        // Apply test inputs
        Xin = $random() % 8192; // Random input within range
        Yin = ($random() & 1) ? -Xin : Xin; // Variable polarity
        $display("Angle = %d, Xin = %d, Yin = %d", angle, Xin, Yin);

        // Wait for module to settle
        #(PERIOD * 10); // Adjust based on module's latency

        // Capture and compare outputs
        if (abs(Xout - ($signed($bits(Xout))) * expected_sin) > 1 ||
            abs(Yout - ($signed($bits(Yout))) * expected_cos) > 1) begin
            // Log error
            $fwrite(err_file, "%d,%f,%f,%d,%d,%f\n",
                    angle, expected_sin, expected_cos, Xout, Yout,
                    abs(Xout - ($signed($bits(Xout))) * expected_sin) +
                    abs(Yout - ($signed($bits(Yout))) * expected_cos));
        end
    end

    // Close error log file
    $fclose(err_file);
    $finish;
end

endmodule
*/
module cordic_final_tb;
localparam SZ=16;// bit accuracy

reg [SZ-1:0] Xin,Yin;
reg [31:0] angle;
wire  [SZ:0] Xout,Yout;
reg CLK_100Mhz;
//wave generator code
localparam FALSE =1'b0;
localparam TRUE =1'b1;
localparam VALUE =32000/1.647;


reg signed [63:0] i=0;
reg signed [63:0] j=0;
reg start;
initial 
begin
start=FALSE;
$write("starting simulation");
CLK_100Mhz=1'b0;
angle=0;
Xin=VALUE;
Yin=1'd0;
#10
@(posedge CLK_100Mhz);
start=TRUE;



for(i=0;i<360;i=i+1)
begin 
@(posedge CLK_100Mhz);
start=FALSE;
angle=((1<<32)*i)/360;

$display("angle=%d:%h",i,angle);
end


#500
$write("simulation has finished");



end

cordic_final cf(CLK_100Mhz,angle,Xin,Yin,Xout,Yout);

parameter clk100_speed=10;
initial
begin

CLK_100Mhz=1'b0;
$display("clk_100Mhz started");
#5
forever
begin
#(clk100_speed/2)CLK_100Mhz=1'b1;

#(clk100_speed/2)CLK_100Mhz=1'b0;

end
end

endmodule



/*




module cordic_tb;

localparam SZ = 16; // Adjust data type and bit accuracy as needed

reg [SZ-1:0] Xin, Yin;
reg [31:0] angle;
wire [SZ-1:0] Xout, Yout;
reg CLK_100Mhz;

// Wave generator code (optional)
localparam FALSE = 1'b0;
localparam TRUE = 1'b1;
localparam VALUE = 32000 / 1.647; // Adjust based on data type and scaling

// ... (remove other unused variables)

reg signed [63:0] i = 0;

initial begin
    $write("Starting simulation");
    CLK_100Mhz = 1'b0;
    angle = 0;
    Xin = VALUE;
    Yin = 1'd0;
    #((PERIOD/2) + ((PERIOD-1)/100)); // Potential glitch avoidance

    forever begin
        @(posedge CLK_100Mhz);

        // Angle calculation with accurate scaling
        angle = (i * $pi * 2**29) / (clk100_speed * 360); // ...

        // Wait sufficient time for CORDIC module to complete calculations
        #(PERIOD * 10); // Adjust based on module's latency

        // ... (add optional error checking or output analysis)

        i = i + 1; // Process all angles or stop condition
        if (i >= 360) begin
            break;
        end
    end

    $write("Simulation finished");
    $stop;
end

cordic_final dut (CLK_100Mhz, angle, Xin, Yin, Xout, Yout);

endmodule

*/




















































/*
module Coric_final_tb;

  localparam width = 16; //width of x and y

  // Inputs
  reg [width-1:0] Xin, Yin;
  reg [31:0] angle;
  reg clk;
  reg signed [63:0] i;

  wire [width-1:0] COSout, SINout;

  localparam An = 32000/1.647;//reduce gain of system 

  initial begin

    //set initial values
    angle = 'b00110101010101010101010101010101;
    Xin = An;     // Xout = 32000*cos(angle)
    Yin = 0;      // Yout = 32000*sin(angle)

    //set clock
    clk = 'b0;
    forever
    begin
      #5 clk = !clk;
    end

    #50

    // Test 1
    #1000                                           
    angle = 'b00100000000000000000000000000000;    // example: 45 deg = 45/360 * 2^32 = 32'b00100000000000000000000000000000 = 45.000 degrees -> atan(2^0)

  //// Test 2
  #1500
 angle = 'b00101010101010101010101010101010; // 60 deg

// Test 3
   #10000
   angle = 'b01000000000000000000000000000000; // 90 deg

//   Test 4
    #10000
angle = 'b00110101010101010101010101010101; // 75 deg

   #1000
   $write("Simulation has finished");
   $stop;

  end

  CORDIC TEST_RUN(clk, COSout, SINout, Xin, Yin, angle);

  // Monitor the output
  initial
  $monitor($time, , COSout, , SINout, , angle);

endmodule




// Testbench module for cordic processor
module tb_cordic;

// Declare the parameters and signals
parameter XY_SZ = 16;
localparam STG = XY_SZ;
parameter CLK_PERIOD = 10; // Clock period in ns
reg clock; // Clock signal
reg signed [31:0] angle; // Angle input
reg signed [XY_SZ-1:0] Xin; // X input
reg signed [XY_SZ-1:0] Yin; // Y input
wire signed [XY_SZ:0] Xout; // X output
wire signed [XY_SZ:0] Yout; // Y output

// Instantiate the design under test (DUT)
cordic_final dut (
  .clock(clock),
  .angle(angle),
  .Xin(Xin),
  .Yin(Yin),
  .Xout(Xout),
  .Yout(Yout)
);

// Generate the clock signal
always @(posedge clock) begin
  #(CLK_PERIOD/2) clock = ~clock;
end
// Initialize the inputs
initial begin
  clock = 0; // Initial value of clock
  angle = 0; // Initial value of angle
  Xin = 0; // Initial value of X
  Yin = 0; // Initial value of Y
  #CLK_PERIOD; // Wait for one clock cycle
end

// Apply the test vectors
initial begin
  // Loop through all angles from 0 to 359 degrees
  for (angle = 0; angle < 360; angle = angle + 1) begin
    // Convert the angle from degrees to radians
    // and scale it by 2^30 to fit in 32 bits
    angle = angle * 3.1415926 * (1 << 30) / 180;
    // Set the inputs to the DUT
    Xin = 1 << (XY_SZ - 1); // Set X to the maximum positive value
    Yin = 0; // Set Y to zero
    #CLK_PERIOD; // Wait for one clock cycle
    // Display the inputs and outputs
    $display("Angle = %d, Xin = %d, Yin = %d, Xout = %d, Yout = %d",
      angle, Xin, Yin, Xout, Yout);
  end
  // Terminate the simulation
 
end

endmodule



module cordic_final_tb;
localparam SZ=16;// bit accuracy

wire [SZ-1:0] Xin,Yin; // changed from reg to wire
reg [31:0] angle;
wire  [SZ-1:0] Xout,Yout;
reg CLK_100Mhz;
//wave generator code
localparam FALSE =1'b0;
localparam TRUE =1'b1;
localparam VALUE =32000/1.647;


reg signed [63:0] i=0;
reg start;
initial 
begin
start=FALSE;
$write("starting simulation");
CLK_100Mhz=1'b0;
angle=0;
Xin=VALUE; // added initial value for Xin
Yin=1'd0; // added initial value for Yin
#1000;
@(posedge CLK_100Mhz);


start=TRUE;

for(i=0;i<360;i=i+1)
begin 
@(posedge CLK_100Mhz);
start=FALSE;
angle=((1<<31)*i)/180; // changed from 1<<32 to 1<<31 to use signed representation
// added code to update Xin and Yin for each angle
Xin = VALUE * cos(angle * 3.1415926 / (1 << 31)); // Xin = VALUE * cos(angle)
Yin = VALUE * sin(angle * 3.1415926 / (1 << 31)); // Yin = VALUE * sin(angle)
#1000; // increased the delay to cover the computation time of the DUT
// added code to check the outputs of the DUT
$display("Angle = %d, Xin = %d, Yin = %d, Xout = %d, Yout = %d",
  angle, Xin, Yin, Xout, Yout);
$display("Expected Xout = %d, Expected Yout = %d",
  VALUE * cos(angle * 3.1415926 / (1 << 31)), // expected Xout = VALUE * cos(angle)
  VALUE * sin(angle * 3.1415926 / (1 << 31))); // expected Yout = VALUE * sin(angle)
end
#500
$write("simulation has finished");
$stop;


end

cordic_final cf(CLK_100Mhz,angle,Xin,Yin,Xout,Yout);

parameter clk100_speed=100;
initial
begin

CLK_100Mhz=1'b0;
$display("clk_100Mhz started");
#5
forever
begin
#(clk100_speed/2)CLK_100Mhz=1'b1; // used the parameter to set the clock period

#(clk100_speed/2)CLK_100Mhz=1'b0;

end

end

endmodule

*/
