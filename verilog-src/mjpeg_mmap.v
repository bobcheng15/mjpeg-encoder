module MJPEG_MMAP (
    input clk,
    input en,
    input rst_n,
    input [31:0] addr,
    input valid,
    input [3:0] wstrb,
    output reg ready,
    input [31:0] wdata,
    output reg [31:0] rdata,

    // Memory interface
    input mem_ready_0,            //
    output mem_valid_0,           //
    output [31:0] mem_addr_0,     // input_addr
    input [31:0] mem_rdata_0,     // input_rdata
    output mem_write_0,           //
    output [31:0] mem_wdata_0,    //

    input mem_ready_1,            //
    output mem_valid_1,           //
    output [31:0] mem_addr_1,     // output_addr
    input [31:0] mem_rdata_1,     // output_rdata
    output mem_write_1,           // output_wen
    output [31:0] mem_wdata_1     // output_wdata
);

// MJPE_MMAP's address mapping
// TODO 0
// Assign your own memory mapping here
// I have defined some addresses which you may use for your accelerator.
// You can add additional addresses if necessary.

// Keep your memory address alligned to 4 bytes, so you won't run into any trouble.
// Each memory address represents 1 byte, so each MMAP mapping jumps at least 4 bytes = 32bits
// e.g. 0x0, 0x4, 0x8, 0xc
parameter MJPEG_MMAP_BASE = 32'h4000_0000;
parameter MJPEG_MMAP_RANG = 32'h0000_ffff;
parameter MJPEG_MMAP_READ_FINISH    = 32'h0000_0000;
parameter MJPEG_MMAP_WRITE_NUM_COEF = 32'h0000_0004; // jump 4bytes = 32bits
parameter MJPEG_MMAP_INPUT_OFFSET   = 32'h0000_002c;
parameter MJPEG_MMAP_OUTPUT_OFFSET  = 32'h0000_0034;
parameter MJPEG_MMAP_WRITE_START    = 32'h0000_0038;

// Accelerator register
reg start, finish;
reg [6:0] num_coef;
reg [31:0] input_offset, output_offset;
wire Ready;

// Internal masked address
wire [31:0] dnn_addr;
assign dnn_addr = (addr) & MJPEG_MMAP_RANG;

// Accelerator memory port
wire [15:0] input_addr, output_addr;
wire [31:0] output_rdata, output_wdata;
wire [31:0] input_rdata, weight_rdata;
wire output_wen;

// Memory port 1
// TODO 1
// This is for input data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
assign mem_valid_0 = 1;
assign mem_addr_0 = input_addr + input_offset;
assign mem_write_0 = 0;
assign mem_wdata_0 = 0;
assign input_rdata = mem_rdata_0;

// Memory port 2
// TODO 2
// This is for weight data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
assign mem_valid_1 = 1;
assign mem_addr_1 = output_addr * 4 + output_offset;
assign mem_write_1 = 0;
assign mem_wdata_1 = 0;
assign weight_rdata = mem_rdata_1;

/**
 *	This block handles the MMAP request.
 *
 *	Signal description:
 *	[31:0] addr:	MMAP request address
 *	[31:0] wdata:	Data write from master
 *	[31:0] rdata:	Data read from module
 *	[3:0] wstrb:	each bit enables 8-bit write to the 32-bit data
 *	valid:		MMAP request from master
 *	ready:		MMAP request is handled
 */
always @(posedge clk or negedge rst_n)
begin
    if (!rst_n) begin
        start <= 0;
        ready <= 0;
        finish <= 0;
        num_coef <= 0;
        input_offset <= 0;
        output_offset <= 0;
    end else begin
        ready <= 0;
        rdata <= 0;
        finish <= (finish == 0) ? Ready : 1;
        start <= (finish == 1) ? 0 : start;
        if (en && valid && !ready) begin
            if (!wstrb) begin   //read
                case (dnn_addr)
		            // TODO 4
                    // Implement your MMAP read routine here
                    MJPEG_MMAP_READ_FINISH: begin
                        rdata <= finish;
                        ready <= 1;
                        finish <= (finish == 1) ? 0 : finish;
                    end
                    default: $display("MJPEG_MMAP: read invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end else begin      //write
                case (dnn_addr)
		            // TODO 5
                    // Implement your MMAP write routine here
                    MJPEG_MMAP_WRITE_NUM_COEF: begin
                        num_coef <= wdata[6:0];
                        ready <= 1;
                    end
                    MJPEG_MMAP_WRITE_START: begin
                        start <= wdata[0];
                        ready <= 1;
                    end
                    MJPEG_MMAP_INPUT_OFFSET: begin
                        input_offset <= wdata;
                        ready <= 1;
                    end
                    MJPEG_MMAP_OUTPUT_OFFSET: begin
                        output_offset <= wdata;
                        ready <= 1;
                    end

                    default: $display("MJPEG_MMAP: write invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end
        end
    end
end



endmodule
