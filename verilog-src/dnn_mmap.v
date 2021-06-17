module DNN_MMAP (
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
    output [31:0] mem_addr_1,     // weight_addr
    input [31:0] mem_rdata_1,     // weight_rdata
    output mem_write_1,           //
    output [31:0] mem_wdata_1,    //

    input mem_ready_2,            //
    output mem_valid_2,           //
    output [31:0] mem_addr_2,     // output_addr
    input [31:0] mem_rdata_2,     // output_rdata
    output mem_write_2,           // output_wen
    output [31:0] mem_wdata_2     // output_wdata
);

// DNN_MMAP's address mapping
// TODO 0
// Assign your own memory mapping here
// I have defined some addresses which you may use for your accelerator.
// You can add additional addresses if necessary.

// Keep your memory address alligned to 4 bytes, so you won't run into any trouble.
// Each memory address represents 1 byte, so each MMAP mapping jumps at least 4 bytes = 32bits
// e.g. 0x0, 0x4, 0x8, 0xc
parameter DNN_MMAP_BASE = 32'h4000_0000;
parameter DNN_MMAP_RANG = 32'h0000_ffff;
parameter DNN_MMAP_READ_FINISH = 32'h0000_0000;
parameter DNN_MMAP_WRITE_CONV = 32'h0000_0004; // jump 4bytes = 32bits
parameter DNN_MMAP_WRITE_N  =   32'h0000_0008;
parameter DNN_MMAP_WRITE_C  =   32'h0000_000c;
parameter DNN_MMAP_WRITE_H  =   32'h0000_0010;
parameter DNN_MMAP_WRITE_W  =   32'h0000_0014;
parameter DNN_MMAP_WRITE_R  =   32'h0000_0018;
parameter DNN_MMAP_WRITE_S  =   32'h0000_001c;
parameter DNN_MMAP_WRITE_M  =   32'h0000_0020;
parameter DNN_MMAP_WRITE_P  =   32'h0000_0024;
parameter DNN_MMAP_WRITE_Q  =   32'h0000_0028;
parameter DNN_MMAP_INPUT_OFFSET	 = 32'h0000_002c;
parameter DNN_MMAP_WEIGHT_OFFSET = 32'h0000_0030;
parameter DNN_MMAP_OUTPUT_OFFSET = 32'h0000_0034;
parameter DNN_MMAP_WRITE_START   = 32'h0000_0038;

// Accelerator register
reg start, conv, finish;
reg [31:0] N, C, H, W, R, S, M, P, Q;
reg [31:0] input_offset, weight_offset, output_offset;
wire Ready;

// Internal masked address
wire [31:0] dnn_addr;
assign dnn_addr = (addr) & DNN_MMAP_RANG;

// Accelerator memory port
wire [15:0] input_addr, weight_addr, output_addr;
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
assign mem_addr_1 = weight_addr + weight_offset;
assign mem_write_1 = 0;
assign mem_wdata_1 = 0;
assign weight_rdata = mem_rdata_1;

// Memory port 3
// TODO 3
// This is for output data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
// Hint: Since the read/write data is 32bits,
// this memory address will be different to mem_addr_0 and mem_addr_1.
assign mem_valid_2 = 1;
assign mem_addr_2 = 4 * output_addr + output_offset;
assign mem_write_2 = output_wen;
assign mem_wdata_2 = output_wdata;
assign output_rdata = 0;

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
        conv <= 0;
        N <= 0;
        C <= 0;
        H <= 0; 
        W <= 0;
        R <= 0;
        S <= 0;
        M <= 0;
        P <= 0;
        Q <= 0;
        input_offset <= 0;
        weight_offset <= 0;
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
                    DNN_MMAP_READ_FINISH: begin
                        rdata <= finish;
                        ready <= 1;
                        finish <= (finish == 1) ? 0 : finish;
                    end
                    default: $display("DNN_MMAP: read invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end else begin      //write
                case (dnn_addr)
		            // TODO 5
                    // Implement your MMAP write routine here
                    DNN_MMAP_WRITE_CONV: begin
                        conv <= wdata[0];
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_N: begin
                        N <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_C: begin
                        C <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_H: begin
                        H <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_W: begin
                        W <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_R: begin
                        R <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_S: begin
                        S <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_M: begin
                        M <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_P: begin
                        P <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_Q: begin
                        Q <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WRITE_START: begin
                        start <= wdata[0];
                        ready <= 1;
                    end
                    DNN_MMAP_INPUT_OFFSET: begin
                        input_offset <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_WEIGHT_OFFSET: begin
                        weight_offset <= wdata;
                        ready <= 1;
                    end
                    DNN_MMAP_OUTPUT_OFFSET: begin
                        output_offset <= wdata;
                        ready <= 1;
                    end

                    default: $display("DNN_MMAP: write invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end
        end
    end
end

// your homework 3 design
top top01(
    .clk(clk),
    .rst_n(rst_n),
    .valid(start),
    .conv(conv),
    .Wait(), // we only use in pcpi
    .Ready(Ready),
    .input_addr(input_addr),
    .input_rdata(input_rdata),
    .weight_addr(weight_addr),
    .weight_rdata(weight_rdata),
    .output_wen(output_wen),
    .output_addr(output_addr),
    .output_rdata(output_rdata),
    .output_wdata(output_wdata),
    .N(N),
    .C(C),
    .H(H),
    .W(W),
    .R(R),
    .S(S),
    .M(M),
    .P(P),
    .Q(Q)
);

endmodule

module systolic_array(
    input clk,
    input rst_n,
    input clear,
    input [31:0] output_sel,
    input signed [7:0] weight_col_1_i,
    input signed [7:0] weight_col_2_i,
    input signed [7:0] weight_col_3_i,
    input signed [7:0] weight_col_4_i,
    input signed [7:0] input_row_1_i,
    input signed [7:0] input_row_2_i,
    input signed [7:0] input_row_3_i, 
    input signed [7:0] input_row_4_i,
    output reg signed [31:0] output_o
);

integer ii = 0;

// the registers used to delay weight dataflow
// 0~7 first reg, 8~15 second reg, 16~23 third reg
reg signed [23:0] weight_reg_col_1, next_weight_reg_col_1;
reg signed [23:0] weight_reg_col_2, next_weight_reg_col_2;
reg signed [23:0] weight_reg_col_3, next_weight_reg_col_3;
reg signed [23:0] weight_reg_col_4, next_weight_reg_col_4;


// the registers used to delay input dataflow
reg signed [23:0] input_reg_row_1, next_input_reg_row_1;
reg signed [23:0] input_reg_row_2, next_input_reg_row_2;
reg signed [23:0] input_reg_row_3, next_input_reg_row_3;
reg signed [23:0] input_reg_row_4, next_input_reg_row_4;

// the registers used to store the output activation

reg signed [127:0] output_reg_row_1, next_output_reg_row_1;
reg signed [127:0] output_reg_row_2, next_output_reg_row_2;
reg signed [127:0] output_reg_row_3, next_output_reg_row_3;
reg signed [127:0] output_reg_row_4, next_output_reg_row_4;


always@(posedge clk) begin
    if (!rst_n) begin
        weight_reg_col_1 = 24'd0;
        weight_reg_col_2 = 24'd0;
        weight_reg_col_3 = 24'd0;
        weight_reg_col_4 = 24'd0;
        input_reg_row_1 = 24'd0;
        input_reg_row_2 = 24'd0;
        input_reg_row_3 = 24'd0;
        input_reg_row_4 = 24'd0;
        output_reg_row_1 = 128'd0;
        output_reg_row_2 = 128'd0;
        output_reg_row_3 = 128'd0;
        output_reg_row_4 = 128'd0;
    end
    else begin
        weight_reg_col_1 = next_weight_reg_col_1;
        weight_reg_col_2 = next_weight_reg_col_2;
        weight_reg_col_3 = next_weight_reg_col_3;
        weight_reg_col_4 = next_weight_reg_col_4;
        input_reg_row_1 = next_input_reg_row_1;
        input_reg_row_2 = next_input_reg_row_2;
        input_reg_row_3 = next_input_reg_row_3;
        input_reg_row_4 = next_input_reg_row_4;
        output_reg_row_1 = next_output_reg_row_1;
        output_reg_row_2 = next_output_reg_row_2;
        output_reg_row_3 = next_output_reg_row_3;
        output_reg_row_4 = next_output_reg_row_4;
    end
end

always@(*) begin
    // shifting the new weight and the exisiting weights in the weight register
    next_weight_reg_col_1 = {weight_reg_col_1[15:0], weight_col_1_i};
    next_weight_reg_col_2 = {weight_reg_col_2[15:0], weight_col_2_i};
    next_weight_reg_col_3 = {weight_reg_col_3[15:0], weight_col_3_i};
    next_weight_reg_col_4 = {weight_reg_col_4[15:0], weight_col_4_i};

    // shifting the new input activations and the exisiting input activations in the weight register

    next_input_reg_row_1 = {input_reg_row_1[15:0], input_row_1_i};
    next_input_reg_row_2 = {input_reg_row_2[15:0], input_row_2_i};
    next_input_reg_row_3 = {input_reg_row_3[15:0], input_row_3_i};
    next_input_reg_row_4 = {input_reg_row_4[15:0], input_row_4_i};
    
    // calculating the output activation 
    // clear the output activation regs if `clear' is asserted
    if (clear == 1'b1) begin
        next_output_reg_row_1 = 128'd0;
        next_output_reg_row_2 = 128'd0;
        next_output_reg_row_3 = 128'd0;
        next_output_reg_row_4 = 128'd0;
    end else begin
        next_output_reg_row_1[31:0] = $signed(output_reg_row_1[31:0]) + input_row_1_i * weight_col_1_i;
        next_output_reg_row_1[63:32] = $signed(output_reg_row_1[63:32]) + $signed(input_reg_row_1[7:0]) * weight_col_2_i;
        next_output_reg_row_1[95:64] = $signed(output_reg_row_1[95:64]) + $signed(input_reg_row_1[15:8]) * weight_col_3_i;
        next_output_reg_row_1[127:96] = $signed(output_reg_row_1[127:96]) + $signed(input_reg_row_1[23:16]) * weight_col_4_i;
        next_output_reg_row_2[31:0] = $signed(output_reg_row_2[31:0]) + input_row_2_i * $signed(weight_reg_col_1[7:0]);
        next_output_reg_row_2[63:32] = $signed(output_reg_row_2[63:32]) + $signed(input_reg_row_2[7:0]) * $signed(weight_reg_col_2[7:0]);
        next_output_reg_row_2[95:64] = $signed(output_reg_row_2[95:64]) + $signed(input_reg_row_2[15:8]) * $signed(weight_reg_col_3[7:0]);
        next_output_reg_row_2[127:96] = $signed(output_reg_row_2[127:96]) + $signed(input_reg_row_2[23:16]) * $signed(weight_reg_col_4[7:0]);
        next_output_reg_row_3[31:0] = $signed(output_reg_row_3[31:0]) + input_row_3_i * $signed(weight_reg_col_1[15:8]);
        next_output_reg_row_3[63:32] = $signed(output_reg_row_3[63:32]) + $signed(input_reg_row_3[7:0]) * $signed(weight_reg_col_2[15:8]);
        next_output_reg_row_3[95:64] = $signed(output_reg_row_3[95:64]) + $signed(input_reg_row_3[15:8]) * $signed(weight_reg_col_3[15:8]);
        next_output_reg_row_3[127:96] = $signed(output_reg_row_3[127:96]) + $signed(input_reg_row_3[23:16]) * $signed(weight_reg_col_4[15:8]);
        next_output_reg_row_4[31:0] = $signed(output_reg_row_4[31:0]) + input_row_4_i * $signed(weight_reg_col_1[23:16]);
        next_output_reg_row_4[63:32] = $signed(output_reg_row_4[63:32]) + $signed(input_reg_row_4[7:0]) * $signed(weight_reg_col_2[23:16]);
        next_output_reg_row_4[95:64] = $signed(output_reg_row_4[95:64]) + $signed(input_reg_row_4[15:8]) * $signed(weight_reg_col_3[23:16]);
        next_output_reg_row_4[127:96] = $signed(output_reg_row_4[127:96]) + $signed(input_reg_row_4[23:16]) * $signed(weight_reg_col_4[23:16]);
    end
end

always@(*) begin
    case(output_sel)
        32'd0: output_o = output_reg_row_1[31:0];
        32'd1: output_o = output_reg_row_1[63:32];
        32'd2: output_o = output_reg_row_1[95:64];
        32'd3: output_o = output_reg_row_1[127:96];
        32'd4: output_o = output_reg_row_2[31:0];
        32'd5: output_o = output_reg_row_2[63:32];
        32'd6: output_o = output_reg_row_2[95:64];
        32'd7: output_o = output_reg_row_2[127:96];
        32'd8: output_o = output_reg_row_3[31:0];
        32'd9: output_o = output_reg_row_3[63:32];
        32'd10: output_o = output_reg_row_3[95:64];
        32'd11: output_o = output_reg_row_3[127:96];
        32'd12: output_o = output_reg_row_4[31:0];
        32'd13: output_o = output_reg_row_4[63:32];
        32'd14: output_o = output_reg_row_4[95:64];
        32'd15: output_o = output_reg_row_4[127:96];
        default: output_o = 32'd0;
    endcase

end
endmodule

module data_setup_buffer(
    input clk,
    input rst_n,
    input load_en_i,
    input [1:0] load_index_i,
    input [31:0] sram_data_i,
    input [1:0] mask_buffer_data,
    output reg [31:0] orchestrated_data_o
);

// 0~23 1st row/column, 24 ~ 47 2nd row/column, 48 ~ 71 3rd row/column, 72 ~ 95 4th row/column

reg [95:0] orchestrated_data_reg;
reg [95:0] next_orchestrated_data_reg;

always@(posedge clk) begin
    if (!rst_n) begin
        orchestrated_data_reg = 96'd0;
    end else begin
        orchestrated_data_reg = next_orchestrated_data_reg;
    end
end

always@(*) begin
    next_orchestrated_data_reg[23:0] = {8'd0, orchestrated_data_reg[8 +: 16]};
    next_orchestrated_data_reg[47:24] = {8'd0, orchestrated_data_reg[32 +: 16]};
    next_orchestrated_data_reg[71:48] = {8'd0, orchestrated_data_reg[56 +: 16]};
    next_orchestrated_data_reg[95:72] = {8'd0, orchestrated_data_reg[80 +: 16]};
    orchestrated_data_o[7:0] = orchestrated_data_reg[0 +: 8];
    orchestrated_data_o[15:8] = orchestrated_data_reg[24 +: 8];
    orchestrated_data_o[23:16] = orchestrated_data_reg[48 +: 8];
    orchestrated_data_o[31:24] = orchestrated_data_reg[72 +: 8];
    if (load_en_i) begin
        if (load_index_i == 2'b00) begin
            case(mask_buffer_data)
                2'd0: next_orchestrated_data_reg [23:0] = sram_data_i[8 +: 24];
                2'd1: next_orchestrated_data_reg [23:0] = {8'd0, sram_data_i[8 +: 16]};
                2'd2: next_orchestrated_data_reg [23:0] = {16'd0, sram_data_i[8 +: 8]};
                2'd3: next_orchestrated_data_reg [23:0] = 24'd0;
                default: next_orchestrated_data_reg [23:0] = 24'd0;
            endcase
            orchestrated_data_o[7:0] = sram_data_i[0 +: 8];
        end
        else if (load_index_i == 2'b01) begin
            case(mask_buffer_data)
                2'd0: next_orchestrated_data_reg [47:24] = sram_data_i[8 +: 24];
                2'd1: next_orchestrated_data_reg [47:24] = {8'd0, sram_data_i[8 +: 16]};
                2'd2: next_orchestrated_data_reg [47:24] = {16'd0, sram_data_i[8 +: 8]};
                2'd3: next_orchestrated_data_reg [47:24] = 24'd0;
                default: next_orchestrated_data_reg [47:24] = 24'd0;
            endcase
            orchestrated_data_o[15:8] = sram_data_i[0 +: 8];
        end
        else if (load_index_i == 2'b10) begin
            case(mask_buffer_data)
                2'd0: next_orchestrated_data_reg [71:48] = sram_data_i[8 +: 24];
                2'd1: next_orchestrated_data_reg [71:48] = {8'd0, sram_data_i[8 +: 16]};
                2'd2: next_orchestrated_data_reg [71:48] = {16'd0, sram_data_i[8 +: 8]};
                2'd3: next_orchestrated_data_reg [71:48] = 24'd0;
                default: next_orchestrated_data_reg [71:48] = 24'd0;
            endcase
            orchestrated_data_o[23:16] = sram_data_i[0 +: 8];
        end
        else if (load_index_i == 2'b11) begin
            case(mask_buffer_data)
                2'd0: next_orchestrated_data_reg [95:72] = sram_data_i[8 +: 24];
                2'd1: next_orchestrated_data_reg [95:72] = {8'd0, sram_data_i[8 +: 16]};
                2'd2: next_orchestrated_data_reg [95:72] = {16'd0, sram_data_i[8 +: 8]};
                2'd3: next_orchestrated_data_reg [95:72] = 24'd0;
                default: next_orchestrated_data_reg [95:72] = 24'd0;
            endcase
            orchestrated_data_o[31:24] = sram_data_i[0 +: 8];
        end
    end
    //mask the input data if necessary
end

endmodule

module control(
    input clk,
    input rst_n,
    input valid,
    input conv,
    output reg Wait,
    output reg Ready,
    output reg [15:0] input_addr,
    output reg [15:0] weight_addr,
    output reg output_wen,
    output reg [15:0] output_addr,
    output reg load_en,
    output reg [1:0] load_index,
    output reg clear_sys_array,
    output reg [31:0] output_sel,
    output reg [1:0] mask_buffer_data,
    input [31:0] output_rdata,
    input [31:0] N,
    input [31:0] C,
    input [31:0] H,
    input [31:0] W,
    input [31:0] R,
    input [31:0] S,
    input [31:0] M,
    input [31:0] P,
    input [31:0] Q
);


parameter [1:0] IDLE     = 2'b00;
parameter [1:0] CAL_FC   = 2'b01;
parameter [1:0] CAL_CONV = 2'b10;
parameter [1:0] WRITE    = 2'b11;

reg [1:0] state, next_state;
reg [15:0] next_input_addr, next_weight_addr, next_output_addr;
reg next_output_wen;
reg [31:0] count, next_count;
reg [1:0] next_load_index;
reg next_load_en;
reg next_Wait;
reg next_Ready;
reg next_clear_sys_array;
reg [1:0] next_mask_buffer_data;
reg [31:0] conv_count, next_conv_count;
reg [31:0] write_count, next_write_count;
reg [31:0] total_write_count, next_total_write_count;

wire [31:0] inner_product_element_num;
wire [31:0] cal_cyc_count;
wire [31:0] ofmap_num_element;
wire [31:0] kernel_element_num;
wire [31:0] next_input_channal_addr;
wire [31:0] cal_kernel_cyc_count; 


assign inner_product_element_num = (conv)? C * W: H;
assign cal_cyc_count = (conv)? cal_kernel_cyc_count * C: ((inner_product_element_num >> 2) << 2) + |inner_product_element_num[1:0] * 4;
assign ofmap_num_element = (conv)? N * (((M >> 2) << 2) + |M[1:0] * 4) * P * Q: N * M * P * Q; 
// assign ofmap_num_element = 16;
assign next_input_channal_addr = H * W;
assign kernel_element_num = W;
assign cal_kernel_cyc_count  = ((kernel_element_num >> 2) << 2) + (|{kernel_element_num[1:0]}) * 4;

always@(posedge clk) begin
    if (!rst_n || !valid) begin
        state = IDLE;
        input_addr = 16'd0;
        weight_addr = 16'd0;
        output_addr = 16'd0;
        output_wen = 16'd0;
        count = 32'd0;
        load_index = 2'd0;
        load_en = 1'd0;
        Wait = 1'd0;
        Ready = 1'd0;
        clear_sys_array = 1'd0;
        mask_buffer_data = 2'd0;
        conv_count = 32'd0;
        write_count = 32'd0;
        total_write_count = 32'd0;
    end else begin
        #1
        state = next_state;
        weight_addr = next_weight_addr;
        input_addr = next_input_addr;
        output_addr = next_output_addr;
        output_wen = next_output_wen;
        count = next_count;
        load_index = next_load_index;
        load_en = next_load_en;
        Wait = next_Wait;
        Ready = next_Ready;
        clear_sys_array = next_clear_sys_array;
        mask_buffer_data = next_mask_buffer_data;
        conv_count = next_conv_count;
        write_count = next_write_count;
        total_write_count = next_total_write_count;
    end
end

always@(*) begin
    case(state) 
        IDLE: begin
            if (conv)
                next_state = (valid)? CAL_CONV: IDLE;
            else 
                next_state = (valid)? CAL_FC: IDLE;
            next_input_addr = input_addr;
            next_weight_addr = weight_addr;
            next_output_addr = output_addr;
            next_output_wen = output_wen;
            next_count = count;
            next_load_index = load_index;
            next_load_en = load_en;
            next_Wait = (valid)? 1'd1: Wait;
            next_Ready = Ready;
            next_clear_sys_array = 1'd0;
            next_mask_buffer_data = mask_buffer_data;
            next_conv_count = 32'd0;
            next_write_count = 32'd0;
            next_total_write_count = 32'd0;
        end
        CAL_FC: begin
            next_state = (count == cal_cyc_count)? WRITE: CAL_FC;
            if (count >= cal_cyc_count - 1) begin
                next_input_addr = input_addr;
                next_weight_addr = weight_addr;
            end
            else begin
                next_input_addr = (load_index == 2'd2)? input_addr - 3 * inner_product_element_num + 4: input_addr + inner_product_element_num;
                next_weight_addr = (load_index == 2'd2)? weight_addr - 3 * inner_product_element_num + 4: weight_addr + inner_product_element_num;
            end
            next_output_addr = output_addr;
            next_output_wen = (count == cal_cyc_count)? 1'd1: output_wen;
            if (count == 0) begin
                next_load_en = 1'b1;
                next_load_index = 2'd0;
            end
            else if (count == cal_cyc_count) begin
                next_load_en = 1'b0; 
                next_load_index = 2'd0;
            end
            else begin
                next_load_en = load_en;
                next_load_index = load_index + 2'd1;
            end
            next_count = (count == cal_cyc_count)? 32'd0: count + 1;
            next_Wait = Wait;
            next_Ready = Ready;
            next_clear_sys_array = 1'd0;
            next_mask_buffer_data = (count >= cal_cyc_count - 4)?  cal_cyc_count - inner_product_element_num: 2'd0;
            next_conv_count = 32'd0;
            next_write_count = 32'd0;
            next_total_write_count = total_write_count;
        end
        CAL_CONV: begin
            next_state = (count == cal_cyc_count)? WRITE: CAL_CONV;
            if (count >= cal_cyc_count - 1) begin
                next_input_addr = input_addr;
                next_weight_addr = weight_addr;
            end
            else begin
                if (load_index == 2'd2) begin
                    next_input_addr = (conv_count == cal_kernel_cyc_count - 32'd1)? input_addr + 1 - 4 * kernel_element_num + next_input_channal_addr : input_addr - 3 * kernel_element_num + 4;
                    next_weight_addr = (conv_count == cal_kernel_cyc_count - 32'd1)? weight_addr - 3 * inner_product_element_num + 1: weight_addr - 3 * inner_product_element_num + 4;
                end
                else begin
                    next_input_addr = input_addr + kernel_element_num;
                    next_weight_addr = weight_addr + inner_product_element_num;
                end
            end
            next_output_addr = output_addr;
            next_output_wen = (count == cal_cyc_count)? 1'd1: output_wen;
            if (count == 0) begin
                next_load_en = 1'b1;
                next_load_index = 2'd0;
            end else if (count == cal_cyc_count) begin
                next_load_en = 1'b0;
                next_load_index = 2'd0;
            end else begin
                next_load_en = load_en;
                next_load_index = load_index + 2'd1;
            end
            next_count = (count == cal_cyc_count)? 32'd0: count + 32'd1;
            next_Wait = Wait;
            next_Ready = Ready;
            next_clear_sys_array = 1'd0;
            next_mask_buffer_data = (conv_count >= cal_kernel_cyc_count - 32'd4)? cal_kernel_cyc_count - kernel_element_num: 2'd0;
            next_conv_count = (conv_count == cal_kernel_cyc_count - 32'd1)? 32'd0: conv_count + 32'd1;
            next_write_count = write_count;
            next_total_write_count = total_write_count;
        end
        WRITE: begin
            if (total_write_count == ofmap_num_element - 1) begin
                next_state = WRITE;
                next_Ready = 1'd0;
                next_Wait = 1'd1;
                next_count = 32'd0;
                next_output_addr = output_addr + 16'd1;
                next_input_addr = 16'd0;
                next_weight_addr = 16'd0;
                next_output_wen = 1'd1;
                next_load_index = 2'd0;
                next_load_en = 1'd0;
                next_conv_count = 32'd0;
                next_clear_sys_array = 1'd1;
                next_mask_buffer_data = 2'd0;
                next_write_count = 32'd0;
                next_total_write_count = total_write_count + 32'd1;
            end
            else if (total_write_count == ofmap_num_element) begin
                next_state = IDLE;
                next_Ready = 1'd1;
                next_Wait = 1'd0;
                next_count = 32'd0;
                next_output_addr = 16'd0;
                next_weight_addr = 16'd0;
                next_input_addr = 16'd0;
                next_input_addr = 16'd0;
                next_output_wen = 1'd0;
                next_load_index = 2'd0;
                next_load_en = 1'd0;
                next_conv_count = 32'd0;
                next_clear_sys_array = 1'd0;
                next_mask_buffer_data = 2'd0;
                next_write_count = 32'd0; 
                next_total_write_count = total_write_count + 32'd1;
            end
            else begin
                if (conv) begin
                    next_state = (count == 32'd15)? CAL_CONV: WRITE;
                    next_count = (count == 32'd15)? 32'd0: count + 32'd1;
                    if (write_count >= 4 * M - 1) begin
                        next_input_addr = (count == 32'd15)? input_addr - (next_input_channal_addr * (C[15:0] - 16'd1)) + 16'd1: input_addr;
                        next_weight_addr = (count == 32'd15)? 16'd0: weight_addr;
                    end else begin
                        next_input_addr = (count == 32'd15)? input_addr - (next_input_channal_addr * (C - 1)) - 4 * W + 16'd1: input_addr;
                        next_weight_addr = (count == 32'd15)? weight_addr + 32'd1: weight_addr;
                    end
                    next_output_wen = (count == 32'd15)? 1'd0: 1'd1;
                    if (count[1:0] == 2'b11) begin
                        if (count == 32'd15) begin
                            next_output_addr = (write_count >= 4 * M - 1)? output_addr - (((((M >> 2) << 2) + |M[1:0] * 4) - 32'd1) * P) + 16'd1: output_addr - 16'd3 + H;
                        end else begin
                            next_output_addr = output_addr - 3 * H + 1; 
                        end
                    end else begin
                        next_output_addr = output_addr + H;
                    end
                    next_clear_sys_array = (count == 32'd15)? 1'd1: 1'd0;
                    next_write_count = (write_count >= 4 * M - 1 && count == 32'd15)? 32'd0: write_count + 32'd1;
                    next_total_write_count = total_write_count + 32'd1;
                end
                else begin
                    next_state = (count == 32'd3)? CAL_FC: WRITE;
                    next_count = (count == 32'd3)? 32'd0: count + 32'd1;
                    next_input_addr = 16'd0;
                    next_weight_addr = (count == 32'd3)? weight_addr + 16'd4: weight_addr;
                    next_output_wen = (count == 32'd3)? 1'd0: 1'd1;
                    next_output_addr = output_addr + 16'd1;
                    next_clear_sys_array = (count == 32'd3)? 1'd1: 1'd0;
                    next_write_count = 32'd0;
                    next_total_write_count = total_write_count + 32'd1;

                end
                next_Ready = 1'd0;
                next_Wait = 1'd1;
                next_load_en = 1'd0;
                next_load_index = 2'd0;
                next_mask_buffer_data = 2'd0;
                next_conv_count = 32'd0;
            end
        end
        
        default: begin
            next_state = IDLE;
            next_Ready = 1'd0;
            next_Wait = 1'd0;
            next_count = 32'd0;
            next_output_addr =  16'd0;
            next_input_addr = 16'd0;
            next_weight_addr = 16'd0;
            next_load_index = 2'd0;
            next_load_en = 1'd0;
            next_output_addr = 1'd0;
            next_output_wen = 1'd0;
            next_clear_sys_array = 1'd0;
            next_mask_buffer_data = 2'd0;
            next_write_count = 32'd0;

        end
    endcase
end

always@(*) begin
    output_sel = count;
end

endmodule

module top (
    input clk,
    input rst_n,
    input valid,
    input conv,
    output Wait,
    output Ready,
    output [15:0] input_addr,
    input [31:0] input_rdata,
    output [15:0] weight_addr,
    input [31:0] weight_rdata,
    output output_wen,
    output [15:0] output_addr,
    input  signed [31:0] output_rdata,
    output signed [31:0] output_wdata,
    input [31:0] N,
    input [31:0] C,
    input [31:0] H,
    input [31:0] W,
    input [31:0] R,
    input [31:0] S,
    input [31:0] M,
    input [31:0] P,
    input [31:0] Q
);

wire load_en;
wire [1:0] load_index;
wire [1:0] mask_buffer_data;

wire clear_sys_array;
wire [31:0] orchestrated_input;
wire [31:0] orchestrated_weight;
wire [31:0] output_sel;

control ctrl(
    .clk(clk),
    .rst_n(rst_n),
    .valid(valid),
    .conv(conv),
    .Wait(Wait),
    .Ready(Ready),
    .input_addr(input_addr),
    .weight_addr(weight_addr),
    .output_wen(output_wen),
    .output_addr(output_addr),
    .load_en(load_en),
    .load_index(load_index),
    .output_rdata(output_rdata),
    .N(N),
    .C(C),
    .H(H),
    .W(W),
    .R(R),
    .S(S),
    .M(M),
    .P(P),
    .Q(Q),
    .clear_sys_array(clear_sys_array),
    .output_sel(output_sel),
    .mask_buffer_data(mask_buffer_data)
);

data_setup_buffer input_buffer(
    .clk(clk),
    .rst_n(rst_n),
    .load_en_i(load_en),
    .load_index_i(load_index),
    .sram_data_i(input_rdata),
    .orchestrated_data_o(orchestrated_input),
    .mask_buffer_data(mask_buffer_data)
);

data_setup_buffer weight_buffer(
    .clk(clk),
    .rst_n(rst_n),
    .load_en_i(load_en),
    .load_index_i(load_index),
    .sram_data_i(weight_rdata),
    .orchestrated_data_o(orchestrated_weight),
    .mask_buffer_data(mask_buffer_data)
);

systolic_array systolic_datapath(
    .clk(clk),
    .rst_n(rst_n),
    .clear(clear_sys_array),
    .output_sel(output_sel),
    .weight_col_1_i(orchestrated_weight[7:0]),
    .weight_col_2_i(orchestrated_weight[15:8]),
    .weight_col_3_i(orchestrated_weight[23:16]),
    .weight_col_4_i(orchestrated_weight[31:24]),
    .input_row_1_i(orchestrated_input[7:0]),
    .input_row_2_i(orchestrated_input[15:8]),
    .input_row_3_i(orchestrated_input[23:16]),
    .input_row_4_i(orchestrated_input[31:24]),
    .output_o(output_wdata)
);




endmodule


