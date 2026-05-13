/*
 * Protocol parser.
 *
 * Frame format (host -> chip):
 *   [0xAA][LEN][CMD][PAYLOAD...][CRC_LO][CRC_HI][0x55]
 *
 * Frame format (chip -> host):
 *   [0xAA][LEN][STATUS][PAYLOAD...][CRC_LO][CRC_HI][0x55]
 *
 * CRC16-CCITT is computed over: LEN | CMD | PAYLOAD  (or LEN | STATUS | PAYLOAD)
 *
 * Commands:
 *   0x01  PING                 -> echo back, no payload
 *   0x02  GET_VERSION          -> response: 1 byte version + 1 byte chip_id
 *   0x10  WRITE_REG            payload: [ADDR][DATA] -> ack
 *   0x11  READ_REG             payload: [ADDR]        -> response: [DATA]
 *   0x20  WRITE_BLOCK          payload: [ADDR][LEN][DATA...] -> ack (max LEN=64)
 *   0x21  READ_BLOCK           payload: [ADDR][LEN]   -> response: [DATA...]
 *   0x30  START                -> ack (kicks off CXOF engine)
 *   0x31  RESET_ENGINE         -> ack
 *   0x40  GET_STATUS           -> response: [STATUS_BYTE]
 *
 * Response status byte:
 *   0x00 = OK
 *   0x01 = BAD_CRC
 *   0x02 = BAD_FRAME
 *   0x03 = BAD_CMD
 *   0x04 = BUSY
 *   0x05 = ENGINE_ERROR
 *
 * Max payload size: 64 bytes (so total frame max ~ 70 bytes).
 */

`default_nettype none

module protocol_parser (
    input  wire        clk,
    input  wire        rst_n,

    // RX from UART
    input  wire [7:0]  rx_byte,
    input  wire        rx_valid,

    // TX to UART
    output reg  [7:0]  tx_byte,
    output reg         tx_send,
    input  wire        tx_ready,

    // Register file
    output reg         rf_we,
    output reg         rf_re,
    output reg  [7:0]  rf_addr,
    output reg  [7:0]  rf_wdata,
    input  wire [7:0]  rf_rdata,

    // Engine control pulses
    output reg         cmd_start,
    output reg         cmd_reset_eng,

    // Status
    input  wire        engine_busy,
    input  wire        engine_done,
    output reg         protocol_error,
    output wire [1:0]  state_dbg
);

    // ---- frame constants ----
    localparam [7:0] SOF      = 8'hAA;
    localparam [7:0] EOF_BYTE = 8'h55;

    // ---- response status codes ----
    localparam [7:0] ST_OK          = 8'h00;
    localparam [7:0] ST_BAD_CRC     = 8'h01;
    localparam [7:0] ST_BAD_FRAME   = 8'h02;
    localparam [7:0] ST_BAD_CMD     = 8'h03;
    localparam [7:0] ST_BUSY        = 8'h04;
    localparam [7:0] ST_ENGINE_ERR  = 8'h05;

    // ---- commands ----
    localparam [7:0] CMD_PING         = 8'h01;
    localparam [7:0] CMD_GET_VERSION  = 8'h02;
    localparam [7:0] CMD_WRITE_REG    = 8'h10;
    localparam [7:0] CMD_READ_REG     = 8'h11;
    localparam [7:0] CMD_WRITE_BLOCK  = 8'h20;
    localparam [7:0] CMD_READ_BLOCK   = 8'h21;
    localparam [7:0] CMD_START        = 8'h30;
    localparam [7:0] CMD_RESET_ENG    = 8'h31;
    localparam [7:0] CMD_GET_STATUS   = 8'h40;

    // ---- RX FSM states ----
    localparam S_WAIT_SOF    = 4'd0;
    localparam S_WAIT_LEN    = 4'd1;
    localparam S_WAIT_CMD    = 4'd2;
    localparam S_RX_PAYLOAD  = 4'd3;
    localparam S_WAIT_CRC_LO = 4'd4;
    localparam S_WAIT_CRC_HI = 4'd5;
    localparam S_WAIT_EOF    = 4'd6;
    localparam S_DISPATCH    = 4'd7;
    localparam S_BUILD_RESP  = 4'd8;
    localparam S_SEND_RESP   = 4'd9;
    localparam S_ERROR       = 4'd10;
    localparam S_WAIT_RF_READ = 4'd11;

    reg [3:0] state;

    // ---- frame buffers ----
    reg [7:0] frame_len;
    reg [7:0] frame_cmd;
    reg [7:0] payload [0:63];        // max 64 bytes of payload
    reg [6:0] pay_idx;                // 0..63
    reg [7:0] rx_crc_lo, rx_crc_hi;

    // ---- CRC for incoming frame ----
    // Computed over LEN + CMD + PAYLOAD bytes.
    reg [15:0] running_crc;

    function [15:0] crc_step;
        input [15:0] crc_in;
        input [7:0]  data;
        reg   [15:0] c;
        integer i;
        begin
            c = crc_in ^ {data, 8'h00};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[15]) c = (c << 1) ^ 16'h1021;
                else       c = (c << 1);
            end
            crc_step = c;
        end
    endfunction

    // ---- response build state ----
    reg [7:0] resp_payload [0:63];
    reg [6:0] resp_len;
    reg [7:0] resp_status;
    reg [6:0] tx_idx;
    reg [3:0] tx_phase;   // 0=SOF, 1=LEN, 2=STATUS, 3=PAYLOAD..., 4=CRC_LO, 5=CRC_HI, 6=EOF
    reg [15:0] tx_running_crc;

    assign state_dbg = state[1:0];

    // ---- main FSM ----
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_WAIT_SOF;
            frame_len      <= 8'd0;
            frame_cmd      <= 8'd0;
            pay_idx        <= 7'd0;
            rx_crc_lo      <= 8'd0;
            rx_crc_hi      <= 8'd0;
            running_crc    <= 16'hFFFF;
            resp_len       <= 7'd0;
            resp_status    <= ST_OK;
            tx_idx         <= 7'd0;
            tx_phase       <= 4'd0;
            tx_byte        <= 8'd0;
            tx_send        <= 1'b0;
            rf_we          <= 1'b0;
            rf_re          <= 1'b0;
            rf_addr        <= 8'd0;
            rf_wdata       <= 8'd0;
            cmd_start      <= 1'b0;
            cmd_reset_eng  <= 1'b0;
            protocol_error <= 1'b0;
            tx_running_crc <= 16'hFFFF;
            for (i = 0; i < 64; i = i + 1) begin
                payload[i]      <= 8'd0;
                resp_payload[i] <= 8'd0;
            end
        end else begin
            // pulse defaults
            rf_we          <= 1'b0;
            rf_re          <= 1'b0;
            cmd_start      <= 1'b0;
            cmd_reset_eng  <= 1'b0;
            tx_send        <= 1'b0;

            case (state)
                // ---- receive frame ----
                S_WAIT_SOF: begin
                    protocol_error <= 1'b0;
                    if (rx_valid && rx_byte == SOF) begin
                        running_crc <= 16'hFFFF;
                        state       <= S_WAIT_LEN;
                    end
                end

                S_WAIT_LEN: begin
                    if (rx_valid) begin
                        if (rx_byte > 8'd64) begin
                            state <= S_ERROR;       // payload too big
                        end else begin
                            frame_len   <= rx_byte;
                            running_crc <= crc_step(running_crc, rx_byte);
                            pay_idx     <= 7'd0;
                            state       <= S_WAIT_CMD;
                        end
                    end
                end

                S_WAIT_CMD: begin
                    if (rx_valid) begin
                        frame_cmd   <= rx_byte;
                        running_crc <= crc_step(running_crc, rx_byte);
                        if (frame_len == 8'd0) begin
                            state <= S_WAIT_CRC_LO;
                        end else begin
                            state <= S_RX_PAYLOAD;
                        end
                    end
                end

                S_RX_PAYLOAD: begin
                    if (rx_valid) begin
                        payload[pay_idx[5:0]] <= rx_byte;
                        running_crc <= crc_step(running_crc, rx_byte);
                        if (pay_idx + 7'd1 == frame_len[6:0]) begin
                            state <= S_WAIT_CRC_LO;
                        end else begin
                            pay_idx <= pay_idx + 7'd1;
                        end
                    end
                end

                S_WAIT_CRC_LO: begin
                    if (rx_valid) begin
                        rx_crc_lo <= rx_byte;
                        state     <= S_WAIT_CRC_HI;
                    end
                end

                S_WAIT_CRC_HI: begin
                    if (rx_valid) begin
                        rx_crc_hi <= rx_byte;
                        state     <= S_WAIT_EOF;
                    end
                end

                S_WAIT_EOF: begin
                    if (rx_valid) begin
                        if (rx_byte != EOF_BYTE) begin
                            resp_status <= ST_BAD_FRAME;
                            state       <= S_BUILD_RESP;
                        end else if ({rx_crc_hi, rx_crc_lo} != running_crc) begin
                            resp_status <= ST_BAD_CRC;
                            state       <= S_BUILD_RESP;
                        end else begin
                            resp_status <= ST_OK;
                            state       <= S_DISPATCH;
                        end
                    end
                end

                // ---- dispatch command ----
                S_DISPATCH: begin
                    case (frame_cmd)
                        CMD_PING: begin
                            resp_len <= 7'd0;
                            state    <= S_BUILD_RESP;
                        end
                        CMD_GET_VERSION: begin
                            resp_payload[0] <= 8'h01;   // protocol version
                            resp_payload[1] <= 8'hAC;   // chip ID
                            resp_len        <= 7'd2;
                            state           <= S_BUILD_RESP;
                        end
                        CMD_WRITE_REG: begin
                            if (frame_len != 8'd2) begin
                                resp_status <= ST_BAD_FRAME;
                            end else begin
                                rf_addr  <= payload[0];
                                rf_wdata <= payload[1];
                                rf_we    <= 1'b1;
                            end
                            resp_len <= 7'd0;
                            state    <= S_BUILD_RESP;
                        end
                        CMD_READ_REG: begin
                            if (frame_len != 8'd1) begin
                                resp_status <= ST_BAD_FRAME;
                                resp_len    <= 7'd0;
                                state       <= S_BUILD_RESP;
                            end else begin
                                rf_addr <= payload[0];
                                rf_re   <= 1'b1;
                                // register_file rdata is synchronous; wait one cycle
                                resp_len <= 7'd1;
                                state    <= S_WAIT_RF_READ;
                            end
                        end
                        CMD_START: begin
                            if (engine_busy) begin
                                resp_status <= ST_BUSY;
                            end else begin
                                cmd_start <= 1'b1;
                            end
                            resp_len <= 7'd0;
                            state    <= S_BUILD_RESP;
                        end
                        CMD_RESET_ENG: begin
                            cmd_reset_eng <= 1'b1;
                            resp_len      <= 7'd0;
                            state         <= S_BUILD_RESP;
                        end
                        CMD_GET_STATUS: begin
                            rf_addr  <= 8'h00;   // STATUS register
                            rf_re    <= 1'b1;
                            resp_len <= 7'd1;
                            state    <= S_BUILD_RESP;
                        end
                        default: begin
                            resp_status <= ST_BAD_CMD;
                            resp_len    <= 7'd0;
                            state       <= S_BUILD_RESP;
                        end
                    endcase
                end

                S_WAIT_RF_READ: begin
                    // bubble cycle for register_file synchronous read
                    state <= S_BUILD_RESP;
                end

                S_BUILD_RESP: begin
                    // capture rdata if a read was issued in DISPATCH
                    if (frame_cmd == CMD_READ_REG || frame_cmd == CMD_GET_STATUS) begin
                        resp_payload[0] <= rf_rdata;
                    end
                    tx_idx         <= 7'd0;
                    tx_phase       <= 4'd0;
                    tx_running_crc <= 16'hFFFF;
                    state          <= S_SEND_RESP;
                end

                S_SEND_RESP: begin
                    if (tx_ready && !tx_send) begin
                        case (tx_phase)
                            4'd0: begin // SOF
                                tx_byte  <= SOF;
                                tx_send  <= 1'b1;
                                tx_phase <= 4'd1;
                            end
                            4'd1: begin // LEN (resp_len + 1 for status byte)
                                tx_byte        <= {1'b0, resp_len} + 8'd1;
                                tx_running_crc <= crc_step(16'hFFFF, {1'b0, resp_len} + 8'd1);
                                tx_send        <= 1'b1;
                                tx_phase       <= 4'd2;
                            end
                            4'd2: begin // STATUS
                                tx_byte        <= resp_status;
                                tx_running_crc <= crc_step(tx_running_crc, resp_status);
                                tx_send        <= 1'b1;
                                if (resp_len == 7'd0)
                                    tx_phase <= 4'd4;   // jump to CRC
                                else
                                    tx_phase <= 4'd3;
                            end
                            4'd3: begin // PAYLOAD bytes
                                tx_byte        <= resp_payload[tx_idx[5:0]];
                                tx_running_crc <= crc_step(tx_running_crc, resp_payload[tx_idx[5:0]]);
                                tx_send        <= 1'b1;
                                if (tx_idx + 7'd1 == resp_len) begin
                                    tx_phase <= 4'd4;
                                end else begin
                                    tx_idx   <= tx_idx + 7'd1;
                                end
                            end
                            4'd4: begin // CRC LOW
                                tx_byte  <= tx_running_crc[7:0];
                                tx_send  <= 1'b1;
                                tx_phase <= 4'd5;
                            end
                            4'd5: begin // CRC HIGH
                                tx_byte  <= tx_running_crc[15:8];
                                tx_send  <= 1'b1;
                                tx_phase <= 4'd6;
                            end
                            4'd6: begin // EOF
                                tx_byte  <= EOF_BYTE;
                                tx_send  <= 1'b1;
                                tx_phase <= 4'd7;
                            end
                            4'd7: begin // done
                                state <= S_WAIT_SOF;
                            end
                            default: state <= S_WAIT_SOF;
                        endcase
                    end
                end

                S_ERROR: begin
                    protocol_error <= 1'b1;
                    resp_status    <= ST_BAD_FRAME;
                    resp_len       <= 7'd0;
                    state          <= S_BUILD_RESP;
                end

                default: state <= S_WAIT_SOF;
            endcase
        end
    end

endmodule
