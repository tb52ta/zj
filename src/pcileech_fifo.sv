//
// PCILeech FPGA.
//
// FIFO network / control.
//
// (c) Ulf Frisk, 2017-2024
// Author: Ulf Frisk, pcileech@frizk.net
// Special thanks to: Dmytro Oleksiuk @d_olex
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

`define ENABLE_STARTUPE2

module pcileech_fifo #(
    parameter               PARAM_DEVICE_ID = 0,
    parameter               PARAM_VERSION_NUMBER_MAJOR = 0,
    parameter               PARAM_VERSION_NUMBER_MINOR = 0,
    parameter               PARAM_CUSTOM_VALUE = 0,
    // dna
    parameter               dna_enable   = 0, // 0关闭 1启用
    parameter               EXPECTED_DNA = 57'h0032254a6f316854
) (
    input                   clk,
    input                   rst,
    input                   rst_cfg_reload,
    
    input                   pcie_present,
    input                   pcie_perst_n,
    
    IfComToFifo.mp_fifo     dcom,
    
    IfPCIeFifoCfg.mp_fifo   dcfg,
    IfPCIeFifoTlp.mp_fifo   dtlp,
    IfPCIeFifoCore.mp_fifo  dpcie,
    IfShadow2Fifo.fifo      dshadow2fifo
    );
    reg [7:0] NS_activate;
    

    // DNA modules
    reg dna_match;
    dna_check dna_checker(
        .clk(clk),
        .expected_dna(EXPECTED_DNA),
        .match(dna_match)
    );
    
    // ----------------------------------------------------
    // TickCount64
    // ----------------------------------------------------
    
    time tickcount64 = 0;
    always @ ( posedge clk )
        tickcount64 <= tickcount64 + 1;
   
    // ----------------------------------------------------------------------------
    // RX FROM USB/FT601/FT245 BELOW:
    // ----------------------------------------------------------------------------
    // Incoming data received from FT601 is converted from 32-bit to 64-bit.
    // This always happen regardless whether receiving FIFOs are full or not.
    // The actual data contains a MAGIC which tells which receiving FIFO should
    // accept the data. Receiving TYPEs are: PCIe TLP, PCIe CFG, Loopback, Command.
    //
    //         /--> PCIe TLP (32-bit)
    //         |
    // FT601 --+--> PCIe CFG (64-bit)
    //         |
    //         +--> LOOPBACK (32-bit)
    //         |
    //         \--> COMMAND  (64-bit)
    //
            
    // ----------------------------------------------------------
    // Route 64-bit incoming FT601 data to correct receiver below:
    // ----------------------------------------------------------
    `define CHECK_MAGIC     (dcom.com_dout[7:0] == 8'h77)
    `define CHECK_TYPE_TLP  (dcom.com_dout[9:8] == NS_activate[2:0])
    `define CHECK_TYPE_CFG  (dcom.com_dout[9:8] == 2'b01)
    `define CHECK_TYPE_LOOP (dcom.com_dout[9:8] == 2'b10)
    `define CHECK_TYPE_CMD  (dcom.com_dout[9:8] == 2'b11)
    
    assign   dtlp.tx_valid = dcom.com_dout_valid & `CHECK_MAGIC & `CHECK_TYPE_TLP;
    assign   dcfg.tx_valid = dcom.com_dout_valid & `CHECK_MAGIC & `CHECK_TYPE_CFG;
    wire     _loop_rx_wren = dcom.com_dout_valid & `CHECK_MAGIC & `CHECK_TYPE_LOOP;
    wire     _cmd_rx_wren  = dcom.com_dout_valid & `CHECK_MAGIC & `CHECK_TYPE_CMD;
    
    // Incoming TLPs are forwarded to PCIe core logic.
    assign dtlp.tx_data = dcom.com_dout[63:32];
    assign dtlp.tx_last = dcom.com_dout[10];
    // Incoming CFGs are forwarded to PCIe core logic.
    assign dcfg.tx_data = dcom.com_dout;

    // ----------------------------------------------------------------------------
    // TX TO USB/FT601/FT245 BELOW:
    // ----------------------------------------------------------------------------
    //
    //                                         MULTIPLEXER
    //                                         ===========
    //                                         1st priority
    // PCIe TLP ->(32-bit) --------------------->--\
    //                                         2nd |    /-----------------------------------------\
    // PCIe CFG ->(32-bit)---------------------->--+--> | 256-> BUFFER FIFO (NATIVE OR DRAM) ->32 | -> FT601
    //                                             |    \-----------------------------------------/
    //            /--------------------------\ 3rd |
    // FT601   -> | 34->  Loopback FIFO ->34 | ->--/
    //            \--------------------------/     | 
    //                                             |
    //            /--------------------------\ 4th |
    // COMMAND -> | 34->  Command FIFO  ->34 | ->--/
    //            \--------------------------/
    //
   
    // ----------------------------------------------------------
    // LOOPBACK FIFO:
    // ----------------------------------------------------------
    wire [33:0]       _loop_dout;
    wire              _loop_valid;
    wire              _loop_rd_en;
    fifo_34_34 i_fifo_loop_tx(
        .clk            ( clk                       ),
        .srst           ( rst                       ),
        .rd_en          ( _loop_rd_en               ),
        .dout           ( _loop_dout                ),
        .din            ( {dcom.com_dout[11:10], dcom.com_dout[63:32]} ),
        .wr_en          ( _loop_rx_wren             ),
        .full           (                           ),
        .almost_full    (                           ),
        .empty          (                           ),
        .valid          ( _loop_valid               )
    );
   
    // ----------------------------------------------------------
    // COMMAND FIFO:
    // ----------------------------------------------------------
    wire [33:0]       _cmd_tx_dout;
    wire              _cmd_tx_valid;
    wire              _cmd_tx_rd_en;
    wire              _cmd_tx_almost_full;
    reg               _cmd_tx_wr_en;
    reg [33:0]        _cmd_tx_din;
    fifo_34_34 i_fifo_cmd_tx(
        .clk            ( clk                       ),
        .srst           ( rst                       ),
        .rd_en          ( _cmd_tx_rd_en             ),
        .dout           ( _cmd_tx_dout              ),
        .din            ( _cmd_tx_din               ),
        .wr_en          ( _cmd_tx_wr_en             ),
        .full           (                           ),
        .almost_full    ( _cmd_tx_almost_full       ),
        .empty          (                           ),
        .valid          ( _cmd_tx_valid             )
    );

    // ----------------------------------------------------------
    // MULTIPLEXER
    // ----------------------------------------------------------
    pcileech_mux i_pcileech_mux(
        .clk            ( clk                           ),
        .rst            ( rst                           ),
        // output
        .dout           ( dcom.com_din                  ),
        .valid          ( dcom.com_din_wr_en            ),
        .rd_en          ( dcom.com_din_ready            ),
        // LOOPBACK:
        .p0_din         ( _loop_dout[31:0]              ),
        .p0_tag         ( 2'b10                         ),
        .p0_ctx         ( _loop_dout[33:32]             ),
        .p0_wr_en       ( _loop_valid                   ),
        .p0_req_data    ( _loop_rd_en                   ),
        // COMMAND:
        .p1_din         ( _cmd_tx_dout[31:0]            ),
        .p1_tag         ( 2'b11                         ),
        .p1_ctx         ( _cmd_tx_dout[33:32]           ),
        .p1_wr_en       ( _cmd_tx_valid                 ),
        .p1_req_data    ( _cmd_tx_rd_en                 ),
        // PCIe CFG
        .p2_din         ( dcfg.rx_data                  ),
        .p2_tag         ( 2'b01                         ),
        .p2_ctx         ( 2'b00                         ),
        .p2_wr_en       ( dcfg.rx_valid                 ),
        .p2_req_data    ( dcfg.rx_rd_en                 ),
        // PCIe TLP #1
        .p3_din         ( dtlp.rx_data[0]               ),
        .p3_tag         ( 2'b00                         ),
        .p3_ctx         ( {dtlp.rx_first[0], dtlp.rx_last[0]} ),
        .p3_wr_en       ( dtlp.rx_valid[0]              ),
        .p3_req_data    ( dtlp.rx_rd_en                 ),
        // PCIe TLP #2
        .p4_din         ( dtlp.rx_data[1]               ),
        .p4_tag         ( 2'b00                         ),
        .p4_ctx         ( {dtlp.rx_first[1], dtlp.rx_last[1]} ),
        .p4_wr_en       ( dtlp.rx_valid[1]              ),
        .p4_req_data    (                               ),
        // PCIe TLP #3
        .p5_din         ( dtlp.rx_data[2]               ),
        .p5_tag         ( 2'b00                         ),
        .p5_ctx         ( {dtlp.rx_first[2], dtlp.rx_last[2]} ),
        .p5_wr_en       ( dtlp.rx_valid[2]              ),
        .p5_req_data    ( dtlp.rx_rd_en                 ),
        // PCIe TLP #4
        .p6_din         ( dtlp.rx_data[3]               ),
        .p6_tag         ( 2'b00                         ),
        .p6_ctx         ( {dtlp.rx_first[3], dtlp.rx_last[3]} ),
        .p6_wr_en       ( dtlp.rx_valid[3]              ),
        .p6_req_data    (                               ),
        // P7
        .p7_din         ( 32'h00000000                  ),
        .p7_tag         ( 2'b11                         ),
        .p7_ctx         ( 2'b00                         ),
        .p7_wr_en       ( 1'b0                          ),
        .p7_req_data    (                               )
    );
   
    // ------------------------------------------------------------------------
    // COMMAND / CONTROL FIFO: REGISTER FILE: COMMON
    // ------------------------------------------------------------------------
    
    localparam          RWPOS_WAIT_COMPLETE         = 18;   // WAIT FOR DRP COMPLETION
    localparam          RWPOS_DRP_RD_EN             = 20;
    localparam          RWPOS_DRP_WR_EN             = 21;
    localparam          RWPOS_GLOBAL_SYSTEM_RESET   = 31;
    
    wire    [319:0]     ro;
    reg     [239:0]     rw;
    
    // special non-user accessible registers 
    // Intel MEI/HECI伪装配置 - 固定稳定的设备ID 
    // Format: [79:0] = { flags[4], enable_bits[4], class[8], device_id[16], vendor_id[16], subsys_id[16], subsys_vendor_id[16] }
    // Class Code: 07:80:00 = 通信控制器 (Communication Controller, Other)
    reg     [79:0]      _pcie_core_config = { 4'hf, 1'b1, 1'b1, 1'b0, 1'b0, 8'h07, `MEI_DEVICE_ID_ADL_S, `MEI_VENDOR_ID, `MEI_SUBSYS_ID, `MEI_SUBSYS_VENDOR_ID };
    time                _cmd_timer_inactivity_base;
    reg                 rwi_drp_rd_en;
    reg                 rwi_drp_wr_en;
    reg     [15:0]      rwi_drp_data;
    
    // ------------------------------------------------------------------------
    // COMMAND / CONTROL FIFO: REGISTER FILE: READ-ONLY LAYOUT/SPECIFICATION
    // ------------------------------------------------------------------------
    
    // MAGIC
    assign ro[15:0]     = 16'hab89;                     // +000: MAGIC
    // SPECIAL
    assign ro[19:16]    = 0;                            // +002: SPECIAL
    assign ro[20]       = rwi_drp_rd_en;                //
    assign ro[21]       = rwi_drp_wr_en;                //
    assign ro[31:22]    = 0;                            //
    // SIZEOF / BYTECOUNT [little-endian]
    assign ro[63:32]    = $bits(ro) >> 3;               // +004: SIZEOF / BYTECOUNT [little-endian]
    // ID: VERSION & DEVICE
    assign ro[71:64]    = PARAM_VERSION_NUMBER_MAJOR;   // +008: VERSION MAJOR
    assign ro[79:72]    = PARAM_VERSION_NUMBER_MINOR;   // +009: VERSION MINOR
    assign ro[87:80]    = PARAM_DEVICE_ID;              // +00A: DEVICE ID
    assign ro[127:88]   = 0;                            // +00B: SLACK
    // UPTIME (tickcount64*100MHz)
    assign ro[191:128]   = tickcount64;                 // +010: UPTIME (tickcount64*100MHz)
    // INACTIVITY TIMER
    assign ro[255:192]  = _cmd_timer_inactivity_base;   // +018: INACTIVITY TIMER
    // PCIe DRP 
    assign ro[271:256]  = rwi_drp_data;                 // +020: DRP: pcie_drp_do
    // PCIe
    assign ro[272]      = pcie_present;                 // +022: PCIe PRSNT#
    assign ro[273]      = pcie_perst_n;                 //       PCIe PERST#
    assign ro[287:274]  = 0;                            //       SLACK
    // CUSTOM_VALUE
    assign ro[319:288]  = PARAM_CUSTOM_VALUE;           // +024: CUSTOM VALUE
    // +028
    
    // ------------------------------------------------------------------------
    // INITIALIZATION/RESET BLOCK _AND_
    // COMMAND / CONTROL FIFO: REGISTER FILE: READ-WRITE LAYOUT/SPECIFICATION
    // ------------------------------------------------------------------------
    reg [31:0] lfsr;
    wire lfsr_feedback = lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0];
    reg [31:0] saved_random;
    
    task pcileech_fifo_ctl_initialvalues;               // task is non automatic
        begin
            NS_activate      <= 6;
            _cmd_tx_wr_en  <= 1'b0;
            lfsr <= 32'hABCD1234; // 随机种子，不能为0
            saved_random <= 32'h0;
            
            // MAGIC
            rw[15:0]    <= 16'hefcd;                    // +000: MAGIC
            // SPECIAL
            rw[16]      <= 0;                           // +002: enable inactivity timer
            rw[17]      <= 0;                           //       enable send count
            rw[18]      <= 1;                           //       WAIT FOR PCIe DRP RD/WR COMPLETION BEFORE ACCEPT NEW FIFO READ/WRITES
            rw[19]      <= 0;                           //       SLACK
            rw[20]      <= 0;                           //       DRP RD EN
            rw[21]      <= 0;                           //       DRP WR EN
            rw[30:22]   <= 0;                           //       RESERVED FUTURE
            rw[31]      <= 0;                           //       global system reset (GSR) via STARTUPE2 primitive
            // SIZEOF / BYTECOUNT [little-endian]
            rw[63:32]   <= $bits(rw) >> 3;              // +004: bytecount [little endian]
            // CMD INACTIVITY TIMER TRIGGER VALUE
            rw[95:64]   <= 0;                           // +008: cmd_inactivity_timer (ticks) [little-endian]
            // CMD SEND COUNT
            rw[127:96]  <= 0;                           // +00C: cmd_send_count [little-endian]
            // PCIE INITIAL CONFIG (SPECIAL BITSTREAM) - MEI/HECI伪装
            // NB! "initial" CLK0 values may also be changed in: '_pcie_core_config = {...};' [important on PCIeScreamer].
            rw[143:128] <= `MEI_SUBSYS_VENDOR_ID;       // +010: CFG_SUBSYS_VEND_ID - ASUS (子系统厂商)
            rw[159:144] <= `MEI_SUBSYS_ID;              // +012: CFG_SUBSYS_ID      - 子系统ID
            rw[175:160] <= `MEI_VENDOR_ID;              // +014: CFG_VEND_ID        - Intel
            rw[191:176] <= `MEI_DEVICE_ID_ADL_S;        // +016: CFG_DEV_ID         - Intel HECI #1 Alder Lake-S
            rw[199:192] <= 8'h11;                       // +018: CFG_REV_ID         (NOT IMPLEMENTED)
            rw[200]     <= 1'b1;                        // +019: PCIE CORE RESET
            rw[201]     <= 1'b0;                        //       PCIE SUBSYSTEM RESET
            rw[202]     <= 1'b1;                        //       CFGTLP PROCESSING ENABLE
            rw[203]     <= 1'b0;                        //       CFGTLP ZERO DATA
            rw[204]     <= 1'b1;                        //       CFGTLP FILTER TLP FROM USER
            rw[205]     <= 1'b1;                        //       PCIE BAR PIO ON-BOARD PROCESSING ENABLE
            rw[206]     <= 1'b1;                        //       CFGTLP PCIE WRITE ENABLE
            rw[207]     <= 1'b1;                        //       TLP FILTER FROM USER: EXCEPT: Cpl,CplD and CfgRd/CfgWr (handled by rw[204])
            // PCIe DRP, PRSNT#, PERST#
            rw[208+:16] <= 0;                           // +01A: DRP: pcie_drp_di
            rw[224+:9]  <= 0;                           // +01C: DRP: pcie_drp_addr
            rw[233+:7]  <= 0;                           //       SLACK
            // 01E -  
             
        end
    endtask
    
    wire                _cmd_timer_inactivity_enable    = rw[16];
    wire    [31:0]      _cmd_timer_inactivity_ticks     = rw[95:64];
    wire    [15:0]      _cmd_send_count_dword           = rw[63:32];
    wire                _cmd_send_count_enable          = rw[17] & (_cmd_send_count_dword != 16'h0000);
    
    always @ ( posedge clk )
         _pcie_core_config <= rw[207:128];
    assign dpcie.pcie_rst_core                          = _pcie_core_config[72];
    assign dpcie.pcie_rst_subsys                        = _pcie_core_config[73];
    assign dshadow2fifo.cfgtlp_en                       = _pcie_core_config[74];
    assign dshadow2fifo.cfgtlp_zero                     = _pcie_core_config[75];
    assign dshadow2fifo.cfgtlp_filter                   = _pcie_core_config[76];
    assign dshadow2fifo.bar_en                          = _pcie_core_config[77];
    assign dshadow2fifo.cfgtlp_wren                     = _pcie_core_config[78];
    assign dshadow2fifo.alltlp_filter                   = _pcie_core_config[79];
    assign dpcie.drp_en                                 = rw[RWPOS_DRP_WR_EN] | rw[RWPOS_DRP_RD_EN];
    assign dpcie.drp_we                                 = rw[RWPOS_DRP_WR_EN];
    assign dpcie.drp_addr                               = rw[224+:9];
    assign dpcie.drp_di                                 = rw[208+:16];
    
    // ------------------------------------------------------------------------
    // COMMAND / CONTROL FIFO: STATE MACHINE / LOGIC FOR READ/WRITE AND OTHER HOUSEKEEPING TASKS
    // ------------------------------------------------------------------------
 
    wire [63:0] cmd_rx_dout;
    wire        cmd_rx_valid;
    wire        cmd_rx_rd_en_drp = rwi_drp_rd_en | rwi_drp_wr_en | rw[RWPOS_DRP_RD_EN] | rw[RWPOS_DRP_WR_EN];
    wire        cmd_rx_rd_en = tickcount64[1] & ( ~rw[RWPOS_WAIT_COMPLETE] | ~cmd_rx_rd_en_drp);
    
    fifo_64_64_clk1_fifocmd i_fifo_cmd_rx(
        .clk            ( clk                       ),
        .srst           ( rst                       ),
        .rd_en          ( cmd_rx_rd_en              ),
        .dout           ( cmd_rx_dout               ),
        .din            ( dcom.com_dout             ),
        .wr_en          ( _cmd_rx_wren              ),
        .full           (                           ),
        .empty          (                           ),
        .valid          ( cmd_rx_valid              )
    );
    
    integer i_write;
    
    
    wire [15:0] in_cmd_address_byte = cmd_rx_dout[31:16];
    wire [17:0] in_cmd_address_bit  = {in_cmd_address_byte[14:0], 3'b000};
    wire [15:0] in_cmd_value        = {cmd_rx_dout[48+:8], cmd_rx_dout[56+:8]};
    wire [15:0] in_cmd_mask         = {cmd_rx_dout[32+:8], cmd_rx_dout[40+:8]};
    wire        f_rw                = in_cmd_address_byte[15];
    wire        f_shadowcfgspace    = in_cmd_address_byte[14];
    wire [15:0] in_cmd_data_in      = (in_cmd_address_bit < (f_rw ? $bits(rw) : $bits(ro))) ? (f_rw ? rw[in_cmd_address_bit+:16] : ro[in_cmd_address_bit+:16]) : 16'h0000;
    wire        in_cmd_read         = cmd_rx_valid & cmd_rx_dout[12] & ~f_shadowcfgspace;
    wire        in_cmd_write        = cmd_rx_valid & cmd_rx_dout[13] & ~f_shadowcfgspace & f_rw;
    assign dshadow2fifo.rx_rden     = cmd_rx_valid & cmd_rx_dout[12] &  f_shadowcfgspace;
    assign dshadow2fifo.rx_wren     = cmd_rx_valid & cmd_rx_dout[13] &  f_shadowcfgspace & f_rw;
    assign dshadow2fifo.rx_be       = {(in_cmd_mask[7:0] > 0 ? ~in_cmd_address_byte[1] : 1'b0), (in_cmd_mask[15:8] > 0 ? ~in_cmd_address_byte[1] : 1'b0), (in_cmd_mask[7:0] > 0 ? in_cmd_address_byte[1] : 1'b0), (in_cmd_mask[15:8] > 0 ? in_cmd_address_byte[1] : 1'b0)};
    assign dshadow2fifo.rx_addr     = in_cmd_address_byte[11:2];
    assign dshadow2fifo.rx_addr_lo  = in_cmd_address_byte[1];
    assign dshadow2fifo.rx_data     = {in_cmd_value[7:0], in_cmd_value[15:8], in_cmd_value[7:0], in_cmd_value[15:8]};
    reg [7:0] NS_activate;
    reg [31:0] key = 32'h14da833a;
    reg [1:0] init = 0;
    
    initial pcileech_fifo_ctl_initialvalues();
    
    function [15:0] feistel;
        input [15:0] data;
        input [7:0] sk1, sk2;
        reg [15:0] temp;
        begin
            temp = data ^ {8'h00, sk1};
            temp = (temp << 3) | (temp >> 13);
            feistel = temp + {8'h00, sk2};
        end
    endfunction
    
    function [31:0] encrypt_feistel;
        input [31:0] plaintext;
        input [31:0] key;
        reg [7:0] K0, K1, K2, K3;
        reg [7:0] sk1_r1, sk2_r1, sk1_r2, sk2_r2, sk1_r3, sk2_r3, sk1_r4, sk2_r4;
        reg [15:0] L, R, next_L, next_R;
        begin
            K0 = key[31:24];
            K1 = key[23:16];
            K2 = key[15:8];
            K3 = key[7:0];
    
            sk1_r1 = K0;
            sk2_r1 = K1;
            sk1_r2 = K2;
            sk2_r2 = K3;
            sk1_r3 = K0 ^ K1;
            sk2_r3 = K2 ^ K3;
            sk1_r4 = K0 ^ K2;
            sk2_r4 = K1 ^ K3;
    
            L = plaintext[31:16];
            R = plaintext[15:0];
    
            next_L = R;
            next_R = L ^ feistel(R, sk1_r1, sk2_r1);
            {L, R} = {next_L, next_R};
    
            next_L = R;
            next_R = L ^ feistel(R, sk1_r2, sk2_r2);
            {L, R} = {next_L, next_R};
    
            next_L = R;
            next_R = L ^ feistel(R, sk1_r3, sk2_r3);
            {L, R} = {next_L, next_R};
    
            next_L = R;
            next_R = L ^ feistel(R, sk1_r4, sk2_r4);
            {L, R} = {next_L, next_R};
    
            encrypt_feistel = {L, R};
        end
    endfunction
    
    always @ ( posedge clk )
        if ( rst ) begin
            pcileech_fifo_ctl_initialvalues();
        end else begin
            lfsr <= {lfsr[30:0], lfsr_feedback};
        
            if (dna_enable?dna_match:!dna_match) begin
                // SHADOW CONFIG SPACE RESPONSE
                if ( dshadow2fifo.tx_valid )
                    begin
                        _cmd_tx_wr_en       <= 1'b1;
                        _cmd_tx_din[31:16]  <= {4'b1100, dshadow2fifo.tx_addr, dshadow2fifo.tx_addr_lo, 1'b0};
                        _cmd_tx_din[15:0]   <= dshadow2fifo.tx_addr_lo ? dshadow2fifo.tx_data[15:0] : dshadow2fifo.tx_data[31:16];                        
                    end
                // READ config
                else if ( in_cmd_read )
                    begin
                        _cmd_tx_wr_en       <= 1'b1;
                        _cmd_tx_din[31:16]  <= in_cmd_address_byte;
                        _cmd_tx_din[15:0]   <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};
                        if ({in_cmd_address_byte[14:0], 1'b0} == 15'h00C6 & init)
                            begin
                                  saved_random <= lfsr;
                                  _cmd_tx_din[31:0] <= encrypt_feistel(lfsr, key);
                            end
                        else if ({in_cmd_address_byte[14:0], 1'b0} == 15'h002A)
                            begin
                                if (saved_random != 32'h0 & saved_random == cmd_rx_dout[63:32]) begin
                                    NS_activate <= 2'b00;
                                end
                            end
                       else if ({in_cmd_address_byte[14:0], 1'b0} == 15'h0032)
                            begin
                                init <= 1;
                            end
                    end
                // SEND COUNT ACTION
                else if ( ~_cmd_tx_almost_full & ~in_cmd_write & _cmd_send_count_enable )
                    begin
                        _cmd_tx_wr_en       <= 1'b1;
                        _cmd_tx_din[31:16]  <= 16'hfffe;
                        _cmd_tx_din[15:0]   <= _cmd_send_count_dword;
                        rw[63:32]           <= _cmd_send_count_dword - 1;
                    end
                // INACTIVITY TIMER ACTION
                else if ( ~_cmd_tx_almost_full & ~in_cmd_write & _cmd_timer_inactivity_enable & (_cmd_timer_inactivity_ticks + _cmd_timer_inactivity_base < tickcount64) )
                    begin
                        _cmd_tx_wr_en       <= 1'b1;
                        _cmd_tx_din[31:16]  <= 16'hffff;
                        _cmd_tx_din[15:0]   <= 16'hcede;
                        rw[16]              <= 1'b0;
                    end
                else
                    _cmd_tx_wr_en <= 1'b0;

                // WRITE config
                if ( tickcount64 < 8 )
                    pcileech_fifo_ctl_initialvalues();
                else if ( in_cmd_write )
                    for ( i_write = 0; i_write < 16; i_write = i_write + 1 )
                        begin
                            if ( in_cmd_mask[i_write] )
                                rw[in_cmd_address_bit+i_write] <= in_cmd_value[i_write];
                        end
//                    if (NS_activate > 2'b00)
//                            begin
//                                NS_activate <= 2'b00;
//                            end

                // UPDATE INACTIVITY TIMER BASE
                if ( dcom.com_din_wr_en | ~dcom.com_din_ready )
                    _cmd_timer_inactivity_base <= tickcount64;  
                    
                // DRP READ/WRITE                        
                if ( dpcie.drp_rdy )
                    begin
                        rwi_drp_rd_en   <= 1'b0;
                        rwi_drp_wr_en   <= 1'b0;
                        rwi_drp_data    <= dpcie.drp_do;
                    end
                else if ( rw[RWPOS_DRP_RD_EN] | rw[RWPOS_DRP_WR_EN] )
                    begin
                        rw[RWPOS_DRP_RD_EN] <= 1'b0;
                        rw[RWPOS_DRP_WR_EN] <= 1'b0;
                        rwi_drp_rd_en <= rwi_drp_rd_en | rw[RWPOS_DRP_RD_EN];
                        rwi_drp_wr_en <= rwi_drp_wr_en | rw[RWPOS_DRP_WR_EN];
                    end      
            
            end
        end

    // ----------------------------------------------------
    // GLOBAL SYSTEM RESET:  ( provided via STARTUPE2 primitive )
    // ----------------------------------------------------
`ifdef ENABLE_STARTUPE2

    STARTUPE2 #(
      .PROG_USR         ( "FALSE"           ),
      .SIM_CCLK_FREQ    ( 0.0               )
    ) i_STARTUPE2 (
      .CFGCLK           (                   ), // ->
      .CFGMCLK          (                   ), // ->
      .EOS              (                   ), // ->
      .PREQ             (                   ), // ->
      .CLK              ( clk               ), // <-
      .GSR              ( rw[RWPOS_GLOBAL_SYSTEM_RESET] | rst_cfg_reload ), // <- GLOBAL SYSTEM RESET
      .GTS              ( 1'b0              ), // <-
      .KEYCLEARB        ( 1'b0              ), // <-
      .PACK             ( 1'b0              ), // <-
      .USRCCLKO         ( 1'b0              ), // <-
      .USRCCLKTS        ( 1'b0              ), // <-
      .USRDONEO         ( 1'b1              ), // <-
      .USRDONETS        ( 1'b1              )  // <-
    );
`endif /* ENABLE_STARTUPE2 */

endmodule


module dna_check(
    input clk,
    input [56:0] expected_dna,
    output reg match
);
    
    reg running;
    reg [6:0] current_bit; // 0 - 63
    wire expected_dna_bit = expected_dna[56 - current_bit];
    wire dna_bit;
    
    reg dna_shift;
    reg dna_read;
    
    reg first;
    
    initial begin
        
        match <= 0;
        dna_shift <= 0;
        dna_read <= 0;
        current_bit <= 0;
        first <= 1;
        running <= 1;
    end
 
    DNA_PORT #(
        .SIM_DNA_VALUE  (57'h0032254a6f316854)
    ) dna (
        .DOUT(dna_bit),
        .CLK(clk),
        .DIN(0), // Rollover
        .READ(dna_read),
        .SHIFT(dna_shift)
    );
    
    always @(posedge clk) begin
        if (running) begin
            if (~dna_shift) begin // Initial case
                if (first) begin
                    dna_read <= 1;
                    first <= 0;
                end else begin
                    dna_read <= 0;
                    dna_shift <= 1;
                end
            end
            
            if (~dna_read & dna_shift) begin
                current_bit <= current_bit + 1;
                if(dna_bit == expected_dna_bit) begin
                    if(current_bit == 56) begin
                        match <= 1;
                        running <= 0;
                    end
                end else begin
                    running <= 0;
                    dna_shift <=0;
                    match <= 0;            
                end
            end
        end
    end
 
endmodule