`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:03:02 10/20/2016 
// Design Name: 
// Module Name:    prreg 
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
module prreg(
    input EN,
    input rst,
	 input clk,
    input [7:0] cmd, //command code (from master)
    input [7:0] ldb, //low data byte (from master)
    input [7:0] hdb, //high data byte (from aster)
	 input rwbit, 		//read/write bit
	 input dbusy,		//data transmission is ongoing
	 input start,		//start condition
	 input stop,		//stop condition
	 input [5:0] count, //falling i2c clock low edge counter
	 input [7:0] vout,  //vout from device ADC
	 input [7:0] iout,  //iout from device ADC
	 input [7:0] tmp,   //temperature
	 input [7:0] vin,	  //vin from device ADC
	 input unitoff,		//unit is off bit
	 input voutov,			//output overvoltage
	 input ioutoc,			//output overcurrent
	 input vinuv,			//input undervoltage
	 input tmpov,			//over temperature
	 input smbalclr,
	 input [7:0] addr,
	 input arbtr,
	 input sysalert,
	 input rstart,
    output cmdv,
    output ldbv,
    output hdbv,
    output [7:0] ldbw,
    output [7:0] hdbw,
	 output [7:0] vid,
	 output [7:0] statusout,
	 output smbalout,
	 output cmlbitout
    );
//parameter definition, addresses
parameter DEVICEADDRESS = 7'd92;
parameter ARA = 7'b0001100;


//parameter definition, read from device command codes
parameter READ_VOUT = 8'h8B; //read output voltage from device
parameter READ_IOUT = 8'h8C; //read output current from device
parameter READ_TMP = 8'h8D; //read temperature from device
parameter READ_VIN = 8'h88; //read input voltage from device
parameter STATUS_BYTE = 8'h78; //status byte
parameter STATUS_VOUT = 8'h7A; //status byte for vout
parameter STATUS_IOUT = 8'h7B; //status byte for iout
parameter STATUS_INPUT = 8'h7C; //status byte for input voltage
parameter STATUS_TEMPERATURE = 8'h7D; //status byte for temperature
parameter STATUS_CML = 8'h7E; //status byte for communication/data fault
parameter STATUS_OTHER = 8'h7F; //status other, set bit 0 when fault and SMBALERT# is high

//parameter definition, read/write from/to device command codes
parameter VOUT_COMMAND = 8'h21; //program output voltage (vid) via PMBus
parameter SMBALERT_MASK = 8'h1B; //mask SMBALERT command

//parameter definitions, commands with no data transmission
parameter CLEAR_FAULTS = 8'h03;

//Other parameters
parameter VOUT_MAX = 8'd250;



//internal registers for command valid, low data byte valid and high data byte valid
reg intcmdv, intldbv, inthdbv;
//internal registers for low data byte write and high data byte write
reg [7:0] intldbw;
reg [7:0] inthdbw;

//registers that are programmed by PMBus
reg [7:0] intvid;

//register to store the STATUS_OTHER command bits
reg [7:0] st_other;

//register to store the STATUS_BYTE bits
reg [7:0] status;

//internal registers for vout, iout, tmp and vin
//regisers hold last value when device busy signal goes high
reg [7:0] voutw;
reg [7:0] ioutw;
reg [7:0] tmpw;
reg [7:0] vinw;

//register for when too many data bytes are being read or written
reg tmbw; //too many bytes written
reg tmbr; //too many bytes read

//register for when too few bits are sent
reg tfb; //too few bits

//register for write bit without repeated start
reg badrdbit;

//register for communication error
reg cmlbit;

//register for SMBALERT#
reg smbal;

//save previous state of fault signals
reg voutovprev;
reg ioutocprev;
reg vinuvprev;
reg tmpovprev;
reg cmlbitprev;

//detect rising edge of fault signals
reg voutovre;
reg ioutocre;
reg vinuvre;
reg tmpovre;
reg cmlbitre;

//voutov status and SMBALERT related signals
reg voutovst; //vout overvoltage signal that goes to bit 7 when STATUS_VOUT is read, also needed for SMBALERT signal manupulations
reg voutovclr; //clears voutovst
reg voutovalertmask; //masks voutov from asserting SMBALER
reg [7:0] stvoutmaskbyte;

//ioutoc status and SMBALERT related signals
reg ioutocst; //iout overcurrent signal that goes to bit 7 when STATUS_IOUT is read, also needed for SMBALERT signal manupulations
reg ioutocclr; //clears ioutocst
reg ioutocalertmask; //masks ioutoc from asserting SMBALERT
reg [7:0] stioutmaskbyte;

//vinuv status and SMBALERT related signals
reg vinuvst; //vin undervoltage signal that goes to bit 7 when STATUS_INPUT is read, also needed for SMBALERT signal manupulations
reg vinuvclr; //clears vinuvst
reg vinuvalertmask; //masks vinuv from asserting SMBALERT
reg [7:0] stvinmaskbyte;

//tmpov status and SMBALERT related signals
reg tmpovst; //overtemperature signal that goes to bit 7 when STATUS_TEMPERATURE is read, also needed for SMBALERT signal manupulations
reg tmpovclr; //clears tmpovst
reg tmpovalertmask; //masks tmpov from asserting SMBALERT
reg [7:0] sttmpmaskbyte;

//cml status and SMBALERT related signals
reg cmdvst;
reg cmdvclr;
reg cmdvalertmask;
reg datavst;
reg datavclr;
reg datavalertmask;
reg otherfltst;
reg otherfltclr;
reg otherfltalertmask;
reg cmdvprev;
reg datavprev;
reg otherfltprev;
reg cmlbitst;
reg [7:0] stcmlmaskbyte;

//STATUS_OTHER bit
reg stother0;



always @(posedge clk or negedge rst)
begin
	if(!rst)
	begin
		intcmdv <= 1'b0;
		intldbv <= 1'b0;
		inthdbv <= 1'b0;
		tmbw <= 1'b0;
		tmbr <= 1'b0;
		tfb <= 1'b0;
		intldbw <= 8'd255;
		inthdbw <= 8'd255;
		intvid <= 8'd0;
		status <= 8'd0;
		voutw <= 8'd0;
		ioutw <= 8'd0;
		tmpw <= 8'd0;
		vinw <= 8'd0;
		cmlbit <= 1'b0;
		smbal <= 1'b0;
		voutovprev <= 1'b0;
		ioutocprev <= 1'b0;
		vinuvprev <= 1'b0;
		tmpovprev <= 1'b0;
		cmlbitprev <= 1'b0;
		voutovre <= 1'b0;
		ioutocre <= 1'b0;
		vinuvre <= 1'b0;
		tmpovre <= 1'b0;
		cmlbitre <= 1'b0;
		voutovst <= 1'b0;
		voutovclr <= 1'b0;
		voutovalertmask <= 1'b0;
		ioutocst <= 1'b0;
		ioutocclr <= 1'b0;
		ioutocalertmask <= 1'b0;
		vinuvst <= 1'b0;
		vinuvclr <= 1'b0;
		vinuvalertmask <= 1'b0;
		tmpovst <= 1'b0;
		tmpovclr <= 1'b0;
		tmpovalertmask <= 1'b0;
		cmdvst <= 1'b0;
		cmdvclr <= 1'b0;
		cmdvalertmask <= 1'b0;
		datavst <= 1'b0;
		datavclr <= 1'b0;
		datavalertmask <= 1'b0;
		otherfltst <= 1'b0;
		otherfltclr <= 1'b0;
		otherfltalertmask <= 1'b0;
		cmdvprev <= 1'b0;
		datavprev <= 1'b0;
		otherfltprev <= 1'b0;
		cmlbitst <= 1'b0;
		stvoutmaskbyte <= 8'd0;
		stioutmaskbyte <= 8'd0;
		stvinmaskbyte <= 8'd0;
		sttmpmaskbyte <= 8'd0;
		stcmlmaskbyte <= 8'd0;
		stother0 <= 1'b0;
		badrdbit <= 1'b0;
	end
	else
	begin
	if(!EN)
	begin
		intcmdv <= 1'b0;
		intldbv <= 1'b0;
		inthdbv <= 1'b0;
		tmbw <= 1'b0;
		tmbr <= 1'b0;
		tfb <= 1'b0;
		intldbw <= 8'd255;
		inthdbw <= 8'd255;
		intvid <= 8'd0;
		status <= 8'd0;
		voutw <= 8'd0;
		ioutw <= 8'd0;
		tmpw <= 8'd0;
		vinw <= 8'd0;
		cmlbit <= 1'b0;
		smbal <= 1'b0;
		voutovprev <= 1'b0;
		ioutocprev <= 1'b0;
		vinuvprev <= 1'b0;
		tmpovprev <= 1'b0;
		cmlbitprev <= 1'b0;
		voutovre <= 1'b0;
		ioutocre <= 1'b0;
		vinuvre <= 1'b0;
		tmpovre <= 1'b0;
		cmlbitre <= 1'b0;
		voutovst <= 1'b0;
		voutovclr <= 1'b0;
		voutovalertmask <= 1'b0;
		ioutocst <= 1'b0;
		ioutocclr <= 1'b0;
		ioutocalertmask <= 1'b0;
		vinuvst <= 1'b0;
		vinuvclr <= 1'b0;
		vinuvalertmask <= 1'b0;
		tmpovst <= 1'b0;
		tmpovclr <= 1'b0;
		tmpovalertmask <= 1'b0;
		cmdvst <= 1'b0;
		cmdvclr <= 1'b0;
		cmdvalertmask <= 1'b0;
		datavst <= 1'b0;
		datavclr <= 1'b0;
		datavalertmask <= 1'b0;
		otherfltst <= 1'b0;
		otherfltclr <= 1'b0;
		otherfltalertmask <= 1'b0;
		cmdvprev <= 1'b0;
		datavprev <= 1'b0;
		otherfltprev <= 1'b0;
		cmlbitst <= 1'b0;
		stvoutmaskbyte <= 8'd0;
		stioutmaskbyte <= 8'd0;
		stvinmaskbyte <= 8'd0;
		sttmpmaskbyte <= 8'd0;
		stcmlmaskbyte <= 8'd0;
		stother0 <= 1'b0;
		badrdbit <= 1'b0;
	end
	else
	begin
		
		//*******************************************************************
		//Freeze updating the vout, iout, temp and vin registers
		//when transmission is occurring
		if(dbusy)
		begin
			voutw <= voutw;
			ioutw <= ioutw;
			tmpw <= tmpw;
			vinw <= vinw;
		end
		else
		begin
			voutw <= vout;
			ioutw <= iout;
			tmpw <= tmp;
			vinw <= vin;
		end
		//********************************************************************
		
		
		//********************************************************************
		//check to see if command is valid
		if(start || stop)
			intcmdv <= 1'b0;
		else
		begin
			if(count == 6'd18 && addr[7:1] != ARA)
			begin //unsupported command
				if(cmd != READ_VOUT && cmd != READ_IOUT && cmd != READ_TMP && cmd != READ_VIN &&
					cmd != STATUS_BYTE && cmd != STATUS_VOUT && cmd != STATUS_IOUT && cmd != STATUS_INPUT && 
					cmd != STATUS_TEMPERATURE && cmd != STATUS_CML && cmd != STATUS_OTHER &&
					cmd != VOUT_COMMAND && cmd != SMBALERT_MASK && cmd != CLEAR_FAULTS) //add other commands as they are implemented
					intcmdv <= 1'b1;
				else
					intcmdv <= 1'b0;
			end
		end
		//Command valid checked
		
		
		//********************************************************************************
		//check to see if high data byte is valid
		if(count == 6'd36)
		begin
			if(cmd == VOUT_COMMAND && hdb > VOUT_MAX) //data out of range
				inthdbv <= 1'b1;
			else
				inthdbv <= inthdbv;
		end
		else if(start || stop)
			inthdbv <= 1'b0;
		else
			inthdbv <= inthdbv;
		//high data byte valid is checked
		//********************************************************************************
		
		//********************************************************************************
		//Check to see if low data byte is valid
		if(count == 6'd27)
		begin
			if((cmd == SMBALERT_MASK && ldb != STATUS_VOUT) &&
				(cmd == SMBALERT_MASK && ldb != STATUS_IOUT) &&
				(cmd == SMBALERT_MASK && ldb != STATUS_INPUT) &&
				(cmd == SMBALERT_MASK && ldb != STATUS_TEMPERATURE) &&
				(cmd == SMBALERT_MASK && ldb != STATUS_CML) &&
				(cmd == SMBALERT_MASK && ldb != 8'd1))
				intldbv <= 1'b1;
			else
				intldbv <= intldbv;
		end
		else if(start || stop)
			intldbv <= 1'b0;
		else
			intldbv <= intldbv;
		//low data byte valid checked
		
		//******************************************************************************
		//check too see if too many data bytes are being written or read
		if(count >= 6'd38 && rwbit == 1'b0) //too many bytes are written by the master
		begin
			tmbw <= 1'b1;
			tmbr <= tmbr;
		end
		else if(count > 6'd28 && rwbit == 1'b1) //too many bytes are read by the master
		begin
			tmbw <= tmbw;
			tmbr <= 1'b1;
		end
		else if(start || stop)
		begin
			tmbw <= 1'b0;
			tmbr <= 1'b0;
		end
		else
		begin
			tmbw <= tmbw;
			tmbr <= tmbr;
		end
		//*****************************************************************************
		
		//*****************************************************************************
		//check to see if too few bits have been sent
		if((stop && !arbtr && !badrdbit) && (
						(count > 6'd1 && count <= 6'd8) || (count > 6'd10 && count <= 6'd17) ||
						(count > 6'd19 && count <= 6'd26) || (count > 6'd28 && count <= 6'd35)
					  )
			)
			tfb <= 1'b1;
		else if(start || stop)
			tfb <= 1'b0;
		else
			tfb <= tfb;
		//*****************************************************************************
		
		//*****************************************************************************
		//check to see if master send a read bit without sending a repeated start
		if(rwbit == 1'b1 && rstart == 1'b0 && dbusy && count >= 6'd9 && addr[7:1] != ARA)
			badrdbit <= 1'b1;
		else if(start || stop)
			badrdbit <= 1'b0;
		else
			badrdbit <= badrdbit;
		
		//setup cml bit
		if(intcmdv || intldbv || inthdbv || tmbw || tmbr || tfb || badrdbit)
			cmlbit <= 1'b1;
		else
			cmlbit <= 1'b0;
		
		
		//*******************************************************************************************************
		//vout overvoltage clear bit
		if((stop && cmd == STATUS_VOUT && ldb[7] == 1'b1 && !cmlbit && addr[0] != 1'b1) || 
			(stop && cmd == CLEAR_FAULTS && addr[7:1] != ARA && !cmlbit))
			voutovclr <= 1'b1;
		else
			voutovclr <= 1'b0;
		
		//setup vout overvoltage fault bit that will go into bit 7 of STATUS_VOUT
		if(voutov == 1'b1 && voutovclr != 1'b1)
			voutovst <= 1'b1;
		else if(voutovclr == 1'b1)
			voutovst <= 1'b0;
		else
			voutovst <= voutovst;
		
		//setup voutovalertmask bit
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_VOUT && !cmlbit)
			voutovalertmask <= hdb[7];
		else
			voutovalertmask <= voutovalertmask;
		
		//store mask byte for STATUS_VOUT
		if(count == 6'd36 && cmd == SMBALERT_MASK && ldb == STATUS_VOUT)
			stvoutmaskbyte <= hdb;
		else
			stvoutmaskbyte <= stvoutmaskbyte;
		
		
		
		//*******************************************************************************************************
		//iout overcurrent clear bit
		if((stop && cmd == STATUS_IOUT && ldb[7] == 1'b1 && !cmlbit && addr[0] != 1'b1) || 
			(stop && cmd == CLEAR_FAULTS && addr[7:1] != ARA && !cmlbit))
			ioutocclr <= 1'b1;
		else
			ioutocclr <= 1'b0;
				
		//setup iout overcurrent fault bit that will go into bit 7 of STATUS_IOUT
		if(ioutoc == 1'b1 && ioutocclr != 1'b1)
			ioutocst <= 1'b1;
		else if(ioutocclr == 1'b1)
			ioutocst <= 1'b0;
		else
			ioutocst <= ioutocst;
		
		//setup ioutocalertmask bit
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_IOUT && !cmlbit)
			ioutocalertmask <= hdb[7];
		else
			ioutocalertmask <= ioutocalertmask;
		
		//store mask byte for STATUS_IOUT
		if(count == 6'd36 && cmd == SMBALERT_MASK && ldb == STATUS_IOUT)
			stioutmaskbyte <= hdb;
		else
			stioutmaskbyte <= stioutmaskbyte;
		
		
		//*******************************************************************************************************
		//vin undervoltage clear bit
		if((stop && cmd == STATUS_INPUT && ldb[4] == 1'b1 && !cmlbit && addr[0] != 1'b1) || 
			(stop && cmd == CLEAR_FAULTS && addr[7:1]!= ARA && !cmlbit))
			vinuvclr <= 1'b1;
		else
			vinuvclr <= 1'b0;
		
		//setup vin undervoltage fault bit that will go into bit 7 of STATUS_INPUT
		if(vinuv == 1'b1 && vinuvclr != 1'b1)
			vinuvst <= 1'b1;
		else if(vinuvclr == 1'b1)
			vinuvst <= 1'b0;
		else
			vinuvst <= vinuvst;
		
		//setup vinuvalertmask bit
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_INPUT && !cmlbit)
			vinuvalertmask <= hdb[4];
		else
			vinuvalertmask <= vinuvalertmask;
		
		//store mask byte for STATUS_INPUT
		if(count == 6'd36 && cmd == SMBALERT_MASK && ldb == STATUS_INPUT)
			stvinmaskbyte <= hdb;
		else
			stvinmaskbyte <= stvinmaskbyte;
		
		
		//*******************************************************************************************************
		//overtemperature clear bit
		if((stop && cmd == STATUS_TEMPERATURE && ldb[7] == 1'b1 && !cmlbit && addr[0] != 1'b1) || 
			(stop && cmd == CLEAR_FAULTS && addr[7:1] != ARA && !cmlbit))
			tmpovclr <= 1'b1;
		else
			tmpovclr <= 1'b0;
		
		//setup overtemperature fault bit that will go into bit 7 of STATUS_TEMPERATURE
		if(tmpov == 1'b1 && tmpovclr != 1'b1)
			tmpovst <= 1'b1;
		else if(tmpovclr == 1'b1)
			tmpovst <= 1'b0;
		else
			tmpovst <= tmpovst;
		
		//setup tmpovalertmask bit
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_TEMPERATURE && !cmlbit)
			tmpovalertmask <= hdb[7];
		else
			tmpovalertmask <= tmpovalertmask;
		
		//store mask byte for STATUS_TEMPERATURE
		if(count == 6'd36 && cmd == SMBALERT_MASK && ldb == STATUS_TEMPERATURE)
			sttmpmaskbyte <= hdb;
		else
			sttmpmaskbyte <= sttmpmaskbyte;
		
		
		//*******************************************************************************************************
		//setup cml clear bit
		//
		if((stop && cmd == STATUS_CML && ldb[7] == 1'b1 && addr[0] != 1'b1) || (stop && cmd == CLEAR_FAULTS))
			cmdvclr <= 1'b1;
		else
			cmdvclr <= 1'b0;
			
		if((stop && cmd == STATUS_CML && ldb[6] == 1'b1 && addr[0] != 1'b1) || (stop && cmd == CLEAR_FAULTS))
			datavclr <= 1'b1;
		else
			datavclr <= 1'b0;
			
		if((stop && cmd == STATUS_CML && ldb[1] == 1'b1 && addr[0] != 1'b1) || (stop && cmd == CLEAR_FAULTS))
			otherfltclr <= 1'b1;
		else
			otherfltclr <= 1'b0;
		
		//setup cml fault bits that will go into STATUS_CML
		if(intcmdv && cmdvclr != 1'b1) //command fault bit
			cmdvst <= 1'b1;
		else if(cmdvclr == 1'b1)
			cmdvst <= 1'b0;
		else
			cmdvst <= cmdvst;
		
		if((intldbv || inthdbv || tmbw) && datavclr != 1'b1) //data content fault bit
			datavst <= 1'b1;
		else if(datavclr == 1'b1)
			datavst <= 1'b0;
		else
			datavst <= datavst;
		
		if((tmbr || tfb || badrdbit) && otherfltclr != 1'b1) //data transmission fault bit
			otherfltst <= 1'b1;
		else if(otherfltclr == 1'b1)
			otherfltst <= 1'b0;
		else
			otherfltst <= otherfltst;
		
		//setup cml bits alert mask bits
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_CML)
			cmdvalertmask <= hdb[7];
		else
			cmdvalertmask <= cmdvalertmask;
			
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_CML)
			datavalertmask <= hdb[6];
		else
			datavalertmask <= datavalertmask;
		
		if(stop && cmd == SMBALERT_MASK && ldb == STATUS_CML)
			otherfltalertmask <= hdb[1];
		else
			otherfltalertmask <= otherfltalertmask;
		
		//store mask byte for STATUS_CML
		if(count == 6'd36 && cmd == SMBALERT_MASK && ldb == STATUS_CML)
			stcmlmaskbyte <= hdb;
		else
			stcmlmaskbyte <= stcmlmaskbyte;
		
		cmlbitst <= cmdvst || datavst || otherfltst;
		
		//store previous values of status bits to detect rising edges for alert and clearing
		voutovprev <= voutovst;
		ioutocprev <= ioutocst;
		vinuvprev <= vinuvst;
		tmpovprev <= tmpovst;
		cmdvprev <= intcmdv;   //cml related bit (command not valid)
		datavprev <= (intldbv || inthdbv || tmbw); //cml related bit (data not valid)
		otherfltprev <= (tmbr || tfb || badrdbit); //cml related bit (transmission fault)
		
		//detect rising edges of faults	
		if((voutovst && !voutovprev) && !voutovalertmask)
			voutovre <= 1'b1;
		else
			voutovre <= 1'b0;
		
		if((ioutocst && !ioutocprev) && !ioutocalertmask)
			ioutocre <= 1'b1;
		else
			ioutocre <= 1'b0;
		
		if((vinuvst && !vinuvprev) && !vinuvalertmask)
			vinuvre <= 1'b1;
		else
			vinuvre <= 1'b0;
		
		if((tmpovst && !tmpovprev)  && !tmpovalertmask)
			tmpovre <= 1'b1;
		else
			tmpovre <= 1'b0;
		
		if((intcmdv && !cmdvprev && !cmdvalertmask) ||
			((intldbv || inthdbv || tmbw) && !datavprev && !datavalertmask) ||
			((tmbr || tfb || badrdbit) && !otherfltprev && !otherfltalertmask))
			cmlbitre <= 1'b1;
		else
			cmlbitre <= 1'b0;
		
		
		//*****************************************************************************************
		//SMBALERT#
		if((cmlbitre || voutovre || ioutocre || vinuvre || tmpovre) && !smbalclr)
			smbal <= 1'b1;
		else if(smbalclr)
			smbal <= 1'b0;
		else
			smbal <= smbal;		
		
		//*****************************************************************************
		//set up status byte
		status <= {1'b0, unitoff, voutovst, ioutocst, vinuvst, tmpovst, cmlbitst, 1'b0};
		//status byte set
		//*******************************************************************************
		
		
		//setup STATUS_OTHER bit
		if(smbal && sysalert)
			stother0 <= 1'b1;
		else if((stop && cmd == STATUS_OTHER && ldb[0] == 1'b1) || (stop && cmd == CLEAR_FAULTS))
			stother0 <= 1'b0;
		else
			stother0 <= stother0;
		
		
		//********************************************************************************
		//take the data from the master and load it to the appropriate registers
		//so far I have vout and status, I will put in more registers after I test functionality		
		//program output voltage via PMBus
		if(stop && cmd == VOUT_COMMAND && cmlbit != 1'b1)
		begin
			if(hdb < VOUT_MAX)
				intvid <= hdb;
			else
				intvid <= intvid;
		end
		else
			intvid <= intvid;
		
		
		//Add other registers down here
		//*****************************************************************
		
		
		
		//*****************************************************************
		//look at write to master commands
		if(intcmdv || badrdbit)
		begin
			intldbw <= 8'd255;
			inthdbw <= 8'd255;
		end
		else
		begin
			if(count == 6'd9 && rwbit == 1'b1 && cmd == VOUT_COMMAND)
			begin
				inthdbw <= intvid;
				intldbw <= 8'd0;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == READ_VOUT)
			begin
				inthdbw <= voutw;
				intldbw <= 8'd0;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == READ_IOUT)
			begin
				inthdbw <= ioutw;
				intldbw <= 8'd0;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == READ_TMP)
			begin
				inthdbw <= tmpw;
				intldbw <= 8'd0;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == READ_VIN)
			begin
				inthdbw <= vinw;
				intldbw <= 8'd0;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_BYTE)
			begin
				intldbw <= status;
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_VOUT)
			begin
				intldbw <= {voutovst,7'b0000000};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_IOUT)
			begin
				intldbw <= {ioutocst,7'b0000000};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_INPUT)
			begin
				intldbw <= {3'b000,vinuvst,4'b0000};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_TEMPERATURE)
			begin
				intldbw <= {tmpovst,7'b0000000};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_CML)
			begin
				intldbw <= {cmdvst,datavst,1'b0,1'b0,1'b0,1'b0,otherfltst,1'b0};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == STATUS_OTHER)
			begin
				intldbw <= {7'b0000000,stother0};
				inthdbw <= 8'd255;
			end
			else if(count == 6'd9 && rwbit == 1'b1 && cmd == SMBALERT_MASK)
			begin
				if(hdb == STATUS_VOUT)
				begin
					intldbw <= 8'd1;
					inthdbw <= stvoutmaskbyte;
				end
				else if(hdb == STATUS_IOUT)
				begin
					intldbw <= 8'd1;
					inthdbw <= stioutmaskbyte;
				end
				else if(hdb == STATUS_INPUT)
				begin
					intldbw <= 8'd1;
					inthdbw <= stvinmaskbyte;
				end
				else if(hdb == STATUS_TEMPERATURE)
				begin
					intldbw <= 8'd1;
					inthdbw <= sttmpmaskbyte;
				end
				else if(hdb == STATUS_CML)
				begin
					intldbw <= 8'd1;
					inthdbw <= stcmlmaskbyte;
				end
				else
				begin
					intldbw <= 8'd255;
					inthdbw <= 8'd255;
				end
			end
			
			//Add other registers as they are implemented
			
			else
			begin
				intldbw <= intldbw;
				inthdbw <= inthdbw;
			end
		end
	end
	end
end

assign cmdv = intcmdv;
assign ldbv = intldbv;
assign hdbv = inthdbv;
assign ldbw = intldbw;
assign hdbw = inthdbw;
assign vid = intvid;
assign statusout = status;
assign smbalout = smbal;
assign cmlbitout = cmlbit;

endmodule
