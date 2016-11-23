`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:55:46 03/19/2015 
// Design Name: 
// Module Name:    communication 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module cmm(
    input EN,
	 input rst,
	 input clk,
    input i2cclk,
    input i2cdata,
	 input cmdvalid,
	 input ldbvalid,
	 input hdbvalid,
	 input [7:0] ldbw, //low data byte write (to master)
	 input [7:0] hdbw, //high data byte write (to master)
	 input smbal,
	 input cmlbit,
    output i2cdataout,
	 output [5:0] i2cclkloout,
	 output dbusyout,
	 output [7:0] cmdout,
	 output [7:0] ldbout,
	 output [7:0] hdbout,
	 output startout,
	 output stopout,
	 output rwbit,
	 output arbtrout,
	 output smbalclrout,
	 output [7:0] addrout,
	 output [6:0] daddrout,
	 output rstartout
	 );

parameter DEVICEADDRESS = 7'd92;
parameter ARA = 7'b0001100;
parameter CLEAR_FAULTS = 8'h03;

reg inti2cdataout;

//counters for i2c clock state changes
reg [5:0] i2cclklo;
reg i2cclkedgehi, i2cclkedgelo;
reg i2cdataedgehi, i2cdataedgelo;
reg previ2cclk, previ2cdata;

//received address register
reg [7:0] addr;
//device address register
reg [6:0] daddr = DEVICEADDRESS;
//received command register
reg [7:0] cmd;
//low data byte
reg [7:0] ldb;
//high data byte
reg [7:0] hdb;

//register for arbitration result and SMBALERT# clear
reg arbtr;
reg smbalclr;

//detect when transmission starts and stops
reg start, stop, dbusy; //dbusy is device busy, high when there is data transmission,
reg rstart;					//low when stop is detected or a byte is NACK-ed by device

//register to load ldbw or daddr
reg [7:0] ldbwout;

always @ (posedge clk or negedge rst)
begin
		if(!rst) //reset mode
		begin
			inti2cdataout <= 1'b1;
			i2cclklo <= 6'd0;
			i2cclkedgehi <= 1'b0;
			i2cclkedgelo <= 1'b0;
			i2cdataedgehi <= 1'b0;
			i2cdataedgelo <= 1'b0;
			previ2cclk <= 1'b1;
			previ2cdata <= 1'b1;
			start <= 1'b0;
			stop <= 1'b0;
			dbusy <= 1'b0;
			addr <= 8'd0;
			cmd <= 8'd0;
			ldb <= 8'd0;
			hdb <= 8'd0;
			rstart <= 1'b0;
			arbtr <= 1'b0;
			smbalclr <= 1'b0;
			ldbwout <= 8'd0;
		end
		else
		begin
		if(!EN)
		begin
			inti2cdataout <= 1'b1;
			i2cclklo <= 6'd0;
			i2cclkedgehi <= 1'b0;
			i2cclkedgelo <= 1'b0;
			i2cdataedgehi <= 1'b0;
			i2cdataedgelo <= 1'b0;
			previ2cclk <= 1'b1;
			previ2cdata <= 1'b1;
			start <= 1'b0;
			stop <= 1'b0;
			dbusy <= 1'b0;
			addr <= 8'd0;
			cmd <= 8'd0;
			ldb <= 8'd0;
			hdb <= 8'd0;
			rstart <= 1'b0;
			arbtr <= 1'b0;
			smbalclr <= 1'b0;
			ldbwout <= 8'd0;
		end
		else
		begin
			previ2cclk <= i2cclk;
			previ2cdata <= i2cdata;
			
			//check to see if i2cclk has a positive edge
			if(!previ2cclk && i2cclk)
				i2cclkedgehi <= 1'b1;
			else
				i2cclkedgehi <= 1'b0;
			
			//check to see if i2cclk has a negative edge
			if(previ2cclk && !i2cclk)
				i2cclkedgelo <= 1'b1;
			else
				i2cclkedgelo <= 1'b0;
				
			//check to see if i2cdata has a positive edge
			if(!previ2cdata && i2cdata)
				i2cdataedgehi <= 1'b1;
			else
				i2cdataedgehi <= 1'b0;
				
			//check to see if 12cdata has a negative edge
			if(previ2cdata && !i2cdata)
				i2cdataedgelo <= 1'b1;
			else
				i2cdataedgelo <= 1'b0;
			
			//check for start and stop conditions
			if(i2cclk && i2cdataedgelo)
				start <= 1'b1;
			else
				start <= 1'b0;
			if(i2cclk && i2cdataedgehi)
				stop <= 1'b1;
			else
				stop <= 1'b0;
			
			//check for repeated start condition
			if(i2cclk && i2cdataedgelo && dbusy)
				rstart <= 1'b1;
			else if(stop)
				rstart <= 1'b0;
			else
				rstart <= rstart;
				
			//assert device busy signal when start is detected and deassert when stop and other conditions are detected
			if(start)
				dbusy <= 1'b1;
			else if(stop || 
					 (i2cclklo == 6'd9 && i2cclkedgehi && inti2cdataout == 1'b1) || //address does not match device address or ARA
					 (i2cclklo == 6'd18 && i2cclkedgehi && addr[0] == 1'b0 && inti2cdataout == 1'b1) || //command not valid
					 (i2cclklo == 6'd18 && i2cclkedgehi && addr[0] == 1'b1 && i2cdata == 1'b1) || //master NACKed low data byte from slave
					 (i2cclklo == 6'd27 && i2cclkedgehi && addr[0] == 1'b0 && inti2cdataout == 1'b1) || //low data byte not valid
					 (i2cclklo == 6'd27 && i2cclkedgehi && addr[0] == 1'b1 && i2cdata == 1'b1) || //master NACKed high data byte from slave
					 (i2cclklo == 6'd36 && i2cclkedgehi && addr[0] == 1'b0 && inti2cdataout == 1'b1)  //high data byte not valid
					 )
				dbusy <= 1'b0;
			else
				dbusy <= dbusy;
				
			//when start is detected set bit counter to 0
			//bit counter counts the falling edges of the i2c clock
			if(start || stop || cmlbit || arbtr)
				i2cclklo <= 6'd0;
			else if(i2cclklo < 6'd38 && i2cclkedgelo && dbusy)
				i2cclklo <= i2cclklo + 1'b1;
			else
				i2cclklo <= i2cclklo;
						
			//*********************************************************************
			//Arbitration for SMBALERT#
			//if ARA matches address bits, ACK and start sending DEVICESDDRESS bits
			//check for arbitration by comparing DEVICEADDRESS bits to i2cdata
			//if a DEVICEADDRESS bit is one, but the i2cdata is 0, then arbitration is lost
			if(addr[7:1] == ARA && smbal && addr[0] == 1'b1)
				begin
					if(i2cclklo == 6'd10 && i2cclkedgehi && daddr[6] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd11 && i2cclkedgehi && daddr[5] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd12 && i2cclkedgehi && daddr[4] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd13 && i2cclkedgehi && daddr[3] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd14 && i2cclkedgehi && daddr[2] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd15 && i2cclkedgehi && daddr[1] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else if(i2cclklo == 6'd16 && i2cclkedgehi && daddr[0] == 1'b1 && i2cdata == 1'b0)
						arbtr <= 1'b1; //arbiration lost
					else
						arbtr <= arbtr; //arbitration won
				end
			else if(start)
				arbtr <= 1'b0; //reset arbitration signal
			else
				arbtr <= arbtr;
			
			
			//*********************************************************************************
			//SMBALERT# clear signal
			if((stop && smbal && !arbtr && addr[7:1] == ARA && !cmlbit) || (stop && cmd == CLEAR_FAULTS && !cmlbit))
				smbalclr <= 1'b1;
			//else if(stop && cmd == CLEAR_FAULTS && !cmlbit)
				//smbalclr <= 1'b1;
			else
				smbalclr <= 1'b0;
			
			
			//**********************************************************************************
			//Load the device address bits to send out for arbitration if sent address is ARA,
			//r/w bit is 1 and smbal is asserted. Other wise load value from input
			if(i2cclklo == 6'd9 && addr[7:1] == ARA && smbal && addr[0] == 1'b1)
			begin
				ldbwout[7:1] <= daddr;
				ldbwout[0] <= 1'b0;
			end
			else if(i2cclklo == 6'd9 && !(addr[7:1] == ARA && smbal && addr[0] == 1'b1))
				ldbwout <= ldbw;
			else
				ldbwout <= ldbwout;
			
			
			//*********************************************************************
			//handle i2cdataout, used for ACK/NACK and data transmission from slave to master
			//assert ACK or NACK address, cmd, data
			if(i2cclklo == 6'd9 && addr[7:1] == DEVICEADDRESS)
				inti2cdataout <= 1'b0; //ACK when device address matches sent address
			else if(i2cclklo == 6'd9 && addr[7:1] == ARA && smbal && addr[0] == 1'b1)
				inti2cdataout <= 1'b0; //ACK if sent address is Alert Response Address and SMBALERT is asserted by device
			else if(i2cclklo == 6'd18 && !cmdvalid && addr[0] == 1'b0)
				inti2cdataout <= 1'b0; //ACK if command is valid and there is a write bit, if there is a read bit, master ACKs
			else if(i2cclklo == 6'd27 && addr[0] == 1'b0 && !ldbvalid) 
				inti2cdataout <= 1'b0; //ACK low data byte (ldb). 
			else if(i2cclklo == 6'd36 && addr[0] == 1'b0 && !ldbvalid && !hdbvalid) 
				inti2cdataout <= 1'b0; //ACK high data byte (hdb).
			//if r/w bit is 1, start sending data to the master
			//send ldbw first, hdbw second, count starts at 10...
			else if(i2cclklo == 6'd10 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[7];
			else if(i2cclklo == 6'd11 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[6];
			else if(i2cclklo == 6'd12 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[5];
			else if(i2cclklo == 6'd13 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[4];
			else if(i2cclklo == 6'd14 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[3];
			else if(i2cclklo == 6'd15 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[2];
			else if(i2cclklo == 6'd16 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[1];
			else if(i2cclklo == 6'd17 && addr[0] == 1'b1)
				inti2cdataout <= ldbwout[0];
			//send hdbw bits
			else if(i2cclklo == 6'd19 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[7];
			else if(i2cclklo == 6'd20 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[6];
			else if(i2cclklo == 6'd21 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[5];
			else if(i2cclklo == 6'd22 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[4];
			else if(i2cclklo == 6'd23 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[3];
			else if(i2cclklo == 6'd24 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[2];
			else if(i2cclklo == 6'd25 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[1];
			else if(i2cclklo == 6'd26 && addr[0] == 1'b1)
				inti2cdataout <= hdbw[0];
			else
				inti2cdataout <= 1'b1;
			
			//********************************************************************
			
			
			//********************************************************************
			//start taking in the bits
			//take the first 8 bits, address and the r/w bit
			if(i2cclklo == 6'd1 && i2cclkedgehi)
			begin
				addr[7] <= i2cdata;
				addr[6:0] <= addr[6:0];
			end
			else if(i2cclklo == 6'd2 && i2cclkedgehi)
			begin
				addr[7] <= addr[7];
				addr[6] <= i2cdata;
				addr[5:0] <= addr[5:0];
				
			end
			else if(i2cclklo == 6'd3 && i2cclkedgehi)
			begin
				
				addr[7:6] <= addr[7:6];
				addr[5] <= i2cdata;
				addr[4:0] <= addr[4:0];
				
			end
			else if(i2cclklo == 6'd4 && i2cclkedgehi)
			begin
				
				addr[7:5] <= addr[7:5];
				addr[4] <= i2cdata;
				addr[3:0] <= addr[3:0];
				
			end
			else if(i2cclklo == 6'd5 && i2cclkedgehi)
			begin
				
				addr[7:4] <= addr[7:4];
				addr[3] <= i2cdata;
				addr[2:0] <= addr[2:0];
				
			end
			else if(i2cclklo == 6'd6 && i2cclkedgehi)
			begin
				
				addr[7:3] <= addr[7:3];
				addr[2] <= i2cdata;
				addr[1:0] <= addr[1:0];
				
			end
			else if(i2cclklo == 6'd7 && i2cclkedgehi)
			begin
				
				addr[7:2] <= addr[7:2];
				addr[1] <= i2cdata;
				addr[0] <= addr[0];
				
			end
			else if(i2cclklo == 6'd8 && i2cclkedgehi)
			begin
				
				addr[7:1] <= addr[7:1];
				addr[0] <= i2cdata;
				
			end
			else
				addr <= addr;				
			//device address and r/wbit taken
			//************************************************
			
			
			//the command bits are transfered when the r/w bit is 0, ie write
			//take in command bits
			if(i2cclklo == 6'd10 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7] <= i2cdata;
				cmd[6:0] <= cmd[6:0];
			end
			else if(i2cclklo == 6'd11 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7] <= cmd[7];
				cmd[6] <= i2cdata;
				cmd[5:0] <= cmd[5:0];
			end
			else if(i2cclklo == 6'd12 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:6] <= cmd[7:6];
				cmd[5] <= i2cdata;
				cmd[4:0] <= cmd[4:0];
			end
			else if(i2cclklo == 6'd13 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:5] <= cmd[7:5];
				cmd[4] <= i2cdata;
				cmd[3:0] <= cmd[3:0];
			end
			else if(i2cclklo == 6'd14 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:4] <= cmd[7:4];
				cmd[3] <= i2cdata;
				cmd[2:0] <= cmd[2:0];
			end
			else if(i2cclklo == 6'd15 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:3] <= cmd[7:3];
				cmd[2] <= i2cdata;
				cmd[1:0] <= cmd[1:0];
			end
			else if(i2cclklo == 6'd16 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:2] <= cmd[7:2];
				cmd[1] <= i2cdata;
				cmd[0] <= cmd[0];
			end
			else if(i2cclklo == 6'd17 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				
				cmd[7:1] <= cmd[7:1];
				cmd[0] <= i2cdata;
			end
			else
				cmd <= cmd;
			//command bit data taken
			//*********************************************************
			
			//*********************************************************
			//if r/w bit (addr[0] is a write (0), take in the first data byte
			//low data byte, ldb (output)
			if(i2cclklo == 6'd19 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7] <= i2cdata;
				ldb[6:0] <= ldb[6:0];
			end
			else if(i2cclklo == 6'd20 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7] <= ldb[7];
				ldb[6] <= i2cdata;
				ldb[5:0] <= ldb[5:0];
			end
			else if(i2cclklo == 6'd21 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:6] <= ldb[7:6];
				ldb[5] <= i2cdata;
				ldb[4:0] <= ldb[4:0];
			end
			else if(i2cclklo == 6'd22 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:5] <= ldb[7:5];
				ldb[4] <= i2cdata;
				ldb[3:0] <= ldb[3:0];
			end
			else if(i2cclklo == 6'd23 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:4] <= ldb[7:4];
				ldb[3] <= i2cdata;
				ldb[2:0] <= ldb[2:0];
			end
			else if(i2cclklo == 6'd24 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:3] <= ldb[7:3];
				ldb[2] <= i2cdata;
				ldb[1:0] <= ldb[1:0];
			end
			else if(i2cclklo == 6'd25 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:2] <= ldb[7:2];
				ldb[1] <= i2cdata;
				ldb[0] <= ldb[0];
			end
			else if(i2cclklo == 6'd26 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				ldb[7:1] <= ldb[7:1];
				ldb[0] <= i2cdata;
			end
			else
				ldb <= ldb;
			//first data byte take in
			//*************************************************************
			
			//take in second data byte, high data byte (hdb)
			if(i2cclklo == 6'd28 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7] <= i2cdata;
				hdb[6:0] <= hdb[6:0];
			end
			else if(i2cclklo == 6'd29 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7] <= hdb[7];
				hdb[6] <= i2cdata;
				hdb[5:0] <= hdb[5:0];
			end
			else if(i2cclklo == 6'd30 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:6] <= hdb[7:6];
				hdb[5] <= i2cdata;
				hdb[4:0] <= hdb[4:0];
			end
			else if(i2cclklo == 6'd31 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:5] <= hdb[7:5];
				hdb[4] <= i2cdata;
				hdb[3:0] <= hdb[3:0];
			end
			else if(i2cclklo == 6'd32 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:4] <= hdb[7:4];
				hdb[3] <= i2cdata;
				hdb[2:0] <= hdb[2:0];
			end
			else if(i2cclklo == 6'd33 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:3] <= hdb[7:3];
				hdb[2] <= i2cdata;
				hdb[1:0] <= hdb[1:0];
			end
			else if(i2cclklo == 6'd34 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:2] <= hdb[7:2];
				hdb[1] <= i2cdata;
				hdb[0] <= hdb[0];
			end
			else if(i2cclklo == 6'd35 && i2cclkedgehi && addr[0] == 1'b0)
			begin
				hdb[7:1] <= hdb[7:1];
				hdb[0] <= i2cdata;
			end
			else
				hdb <= hdb;
			//high data byte taken
			//*************************************************************			
		
		end
		end
end

assign i2cdataout = ~inti2cdataout;
assign i2cclkloout = i2cclklo;
assign dbusyout = dbusy;
assign cmdout = cmd;
assign ldbout = ldb;
assign hdbout = hdb;
assign startout = start;
assign stopout = stop;
assign rwbit = addr[0];
assign arbtrout = arbtr;
assign smbalclrout = smbalclr;
assign addrout = addr;
assign daddrout = daddr;
assign rstartout = rstart;

endmodule
