#include<iostream>
#include<string>
#include<vector>
#include<bitset>
#include<fstream>
#include <filesystem>
#include <typeinfo>
#include <map>
#include <utility>

using namespace std;
namespace fs = std::filesystem;

#define MemSize 1000 // memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.

struct IFStruct {
    bitset<32>  PC;
    bool        nop;  
};

struct IDStruct {
    bitset<32>  Instr;
    bool        nop;  
};

struct EXStruct {
    bitset<32>  Read_data1;
    bitset<32>  Read_data2;
    bitset<16>  Imm;
    bitset<5>   Rs;
    bitset<5>   Rt;
    bitset<5>   Wrt_reg_addr;
    bool        is_I_type;
    bool        rd_mem;
    bool        wrt_mem; 
    bool        alu_op;     //1 for addu, lw, sw, 0 for subu 
    bool        wrt_enable;
    bool        nop;  
};

struct MEMStruct {
    bitset<32>  ALUresult;
    bitset<32>  Store_data;
    bitset<5>   Rs;
    bitset<5>   Rt;    
    bitset<5>   Wrt_reg_addr;
    bool        rd_mem;
    bool        wrt_mem; 
    bool        wrt_enable;    
    bool        nop;    
};

struct WBStruct {
    bitset<32>  Wrt_data;
    bitset<5>   Rs;
    bitset<5>   Rt;     
    bitset<5>   Wrt_reg_addr;
    bool        wrt_enable;
    bool        nop;     
};

struct stateStruct {
    IFStruct    IF;
    IDStruct    ID;
    EXStruct    EX;
    MEMStruct   MEM;
    WBStruct    WB;
};

class InsMem
{
	public:
		string id, ioDir;
        InsMem(string name, string ioDir) {       
			id = name;
			IMem.resize(MemSize);
            ifstream imem;
			string line;
			int i=0;
			imem.open(ioDir + "/imem.txt");
			if (imem.is_open())
			{
				while (getline(imem,line))
				{      
					line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
					IMem[i] = bitset<8>(line);
					i++;
				}                    
			}
            else cout<<"Unable to open IMEM input file.";
			imem.close();                     
		}

		bitset<32> readInstr(bitset<32> ReadAddress) {
			int address = (int)(ReadAddress.to_ulong()); // Convert the bitset to an integer to use as an index

			// Ensure the address is within the bounds of the IMem
			if(address < 0 || address + 3 >= IMem.size()) {
				cout << "Error: Address out of bounds." << endl;
				return bitset<32>(0);  // Return a 0 instruction for error
			}

			// Read the 32-bit instruction from IMem in Big-Endian format
			bitset<32> instr(
				IMem[address].to_string() + 
				IMem[address + 1].to_string() + 
				IMem[address + 2].to_string() + 
				IMem[address + 3].to_string()
			);

			return instr;
		}

      
    private:
        vector<bitset<8> > IMem;     
};
      
class DataMem    
{
    public: 
		string id, opFilePath, ioDir;
        DataMem(string name, string ioDir) : id(name), ioDir(ioDir) {
            DMem.resize(MemSize);
			opFilePath = ioDir + "/" + name + "_DMEMResult.txt";
            ifstream dmem;
            string line;
            int i=0;
            dmem.open(ioDir + "/dmem.txt");
            if (dmem.is_open())
            {
                while (getline(dmem,line))
                {      
					line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
                    DMem[i] = bitset<8>(line);
                    i++;
                }
            }
            else cout<<"Unable to open DMEM input file.";
                dmem.close();          
        }
		
        bitset<32> readDataMem(bitset<32> Address) {	
			int address = (int)(Address.to_ulong()); // Convert the bitset to an integer to use as an index
			// Ensure the address is within the bounds of the IMem
			if(address < 0 || address + 3 >= DMem.size()) {
				cout << "Error: Address out of bounds." << endl;
				return bitset<32>(0);  // Return a 0 instruction for error
			}

			// Read the 32-bit instruction from IMem in Big-Endian format
			bitset<32> instr(
				DMem[address].to_string() + 
				DMem[address + 1].to_string() + 
				DMem[address + 2].to_string() + 
				DMem[address + 3].to_string()
			);

			return instr;
		}
            
        void writeDataMem(bitset<32> Address, bitset<32> WriteData) {
			int addr = (int)(Address.to_ulong());
			for (int i = 3; i >= 0; i--) {
				DMem[addr + i] = bitset<8>(WriteData.to_ulong() & 0xFF);
				WriteData >>= 8;  // shift right by 8 bits
			}
        }   
                     
        void outputDataMem(string outDirPath) {
			string out(outDirPath + "/" + id + "_DMEMResult.txt");
            ofstream dmemout;
            dmemout.open(out, std::ios_base::trunc);
            if (dmemout.is_open()) {
                for (int j = 0; j< 1000; j++)
                {     
                    dmemout << DMem[j]<<endl;
                }
                     
            }
            else cout<<"Unable to open "<<id<<" DMEM result file." << endl;
            dmemout.close();
        }             

    // private:
		vector<bitset<8> > DMem;      
};

class RegisterFile
{
    public:
		string outputFile;
     	RegisterFile(string ioDir): outputFile (ioDir + "RFResult.txt") {
			Registers.resize(32);  
			Registers[0] = bitset<32> (0);  
        }
	
        bitset<32> readRF(bitset<5> Reg_addr) {   
            int addr = (int)(Reg_addr.to_ulong());
			return Registers[addr];
        }
    
        void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data) {
            int addr = (int)(Reg_addr.to_ulong());
			if (addr != 0){
				Registers[addr] = Wrt_reg_data;
			}
        }
		 
		void outputRF(int cycle, string outDirPath) {
			string out(outDirPath + "_RFResult.txt");
			ofstream rfout;
			if (cycle == 0)
				rfout.open(out, std::ios_base::trunc);
			else 
				rfout.open(out, std::ios_base::app);
			if (rfout.is_open())
			{
                rfout << "----------------------------------------------------------------------" << endl;
				rfout<<"State of RF after executing cycle:"<<cycle<<endl;
				for (int j = 0; j<32; j++)
				{

					rfout << Registers[j]<<endl;
				}
			}
			else cout<<"Unable to open RF output file."<<endl;
			rfout.close();               
		} 
			
	private:
		vector<bitset<32> >Registers;
};

class Core {
	public:
		RegisterFile myRF;
		uint32_t cycle = 0;
		bool halted = false;
		string ioDir;
		struct stateStruct state, nextState;
		InsMem ext_imem;
		DataMem ext_dmem;
		
		Core(string ioDir, InsMem &imem, DataMem &dmem): myRF(ioDir), ioDir(ioDir), ext_imem (imem), ext_dmem (dmem) {}

		virtual void step() {}

		virtual void printState() {}
};

string decodeInstruction(const bitset<32>& instr, EXStruct &ex, RegisterFile& rf, map<int, pair<string, bitset<5>>>& prev_op, int cycle, string id);
void executeInstruction(const EXStruct& ex, MEMStruct& mem, RegisterFile& rf, const string& operation, stateStruct& state);
bitset<32> signExtendImm(const bitset<16>& imm);
void clearControlSignals(stateStruct &nextState);
bitset<32> TwosComplement(bitset<32> num);
bool hasHazard(map<int, pair<string, bitset<5>>>& prev_op, bitset<5> rs1 = 0x1f, bitset<5> rs2 = 0x1f, bitset<5> rd = 0x1f);
void outputPerformanceMetrics(int SS_cycle, int SS_num_instr, int FS_cycle, int FS_num_instr, const string& path);

class SingleStageCore : public Core {
	public:
		SingleStageCore(string ioDir, string outDir_, InsMem &imem, DataMem &dmem): 
			Core(ioDir + "/SS_", imem, dmem), outDir(outDir_) {}

		void step(DataMem& dmem_ss) {
			// Initialization
			if (cycle == 0) {
				clearControlSignals(state);
			}

			if (halted) {
				dmem_ss = ext_dmem;
				myRF.outputRF(cycle, outDir+"/SS");
				printState(state, cycle);
				return ;
			}
			// IF
			if (!state.IF.nop){
				state.ID.Instr = ext_imem.readInstr(state.IF.PC);
			}
			else state.ID.nop = true;

			string operation = "";
			// ID
			if (!state.ID.nop){
				operation = decodeInstruction(state.ID.Instr, state.EX, myRF, prev_op, cycle, "SS");
			}
			else state.EX.nop = true;

			// EX
			if (!state.EX.nop) {
				executeInstruction(state.EX, state.MEM, myRF, operation, state);
			}
			else state.MEM.nop = true;

			// MEM
			if (!state.MEM.nop){
				if (state.MEM.rd_mem){
					// cout << operation << " is now reading memory" << endl;
					state.WB.Wrt_data = ext_dmem.readDataMem(state.MEM.ALUresult);
				}
				else {
					if (operation == "JAL") state.WB.Wrt_data = state.MEM.Store_data;
					else {
						state.WB.Wrt_data = state.MEM.ALUresult;
					}
				}
				if (state.MEM.wrt_mem){
					// cout << state.MEM.Store_data << " is writing to memory location "<< state.MEM.ALUresult << endl;
					ext_dmem.writeDataMem(state.MEM.ALUresult, state.MEM.Store_data);
				}
				state.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;
				state.WB.wrt_enable = state.MEM.wrt_enable;
			}
			else state.WB.nop = true;

			// WB
			if (!state.WB.nop && state.WB.wrt_enable) {
				// cout << "Writing " << state.WB.Wrt_data << " to register " << state.WB.Wrt_reg_addr << endl;
				myRF.writeRF(state.WB.Wrt_reg_addr, state.WB.Wrt_data);
			}

			if (state.ID.Instr.to_ulong() == 0xFFFFFFFF){
				state.IF.nop = state.ID.nop = state.EX.nop = state.MEM.nop = state.WB.nop = halted = true;
			}
			else if (operation == "BEQ" || operation == "BNE" || operation == "JAL"){
				int offset = state.MEM.ALUresult.to_ulong();
				if (state.MEM.ALUresult[31] == 1) offset = -TwosComplement(state.MEM.ALUresult).to_ulong();
				state.IF.PC = bitset<32>(state.IF.PC.to_ulong() + offset);
			}
			else {
				state.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
			}
			
			myRF.outputRF(cycle, outDir+"/SS"); // dump RF
			printState(state, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ... 

			// set next state
			nextState.IF = state.IF;
			nextState.ID = state.ID;
			nextState.EX = state.EX;
			nextState.MEM = state.MEM;
			nextState.WB = state.WB;

			clearControlSignals(nextState);
			
			state = nextState; // The end of the cycle and updates the current state with the values calculated in this cycle
			cycle++;
		}

		void printState(stateStruct state, int cycle) {
			string out(outDir + "/StateResult_SS.txt");
    		ofstream printstate;
			if (cycle == 0)
				printstate.open(out, std::ios_base::trunc);
			else 
    			printstate.open(out, std::ios_base::app);
			printstate << "----------------------------------------------------------------------" << endl;
    		if (printstate.is_open()) {
    		    printstate<<"State after executing cycle: "<<cycle<<endl; 

    		    printstate<<"IF.PC: "<<state.IF.PC.to_ulong()<<endl;
    		    printstate<<"IF.nop: "<< (state.IF.nop ? "True" : "False") <<endl;
    		}
    		else cout<<"Unable to open SS StateResult output file." << endl;
    		printstate.close();
		}
	private:
		string outDir;
        map<int, pair<string, bitset<5>>> prev_op;
};

class FiveStageCore : public Core {
	public:
		FiveStageCore(string ioDir, string outDir_, InsMem &imem, DataMem &dmem): 
			Core(ioDir + "/FS_", imem, dmem), outDir(outDir_) {}

		void step(DataMem& dmem_fs) {
			// Initialization
			if (cycle == 0) {
				clearControlSignals(state);
			}

            if (cycle < 4){
                myRF.outputRF(cycle, outDir+"/FS"); // dump RF
			    printState(state, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ... 
                cycle++;
                return ;
            }

			if (halted) {
				dmem_fs = ext_dmem;
				return ;
			}
			// IF
			if (!state.IF.nop){
				state.ID.Instr = ext_imem.readInstr(state.IF.PC);
			}
			else state.ID.nop = true;

			string operation = "";
			// ID
			if (!state.ID.nop){
				operation = decodeInstruction(state.ID.Instr, state.EX, myRF, prev_op, cycle, "FS");
                if (operation == "hazard"){
                    myRF.outputRF(cycle, outDir+"/FS"); // dump RF
			        printState(state, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ... 
                    cycle++;
                    return ;
                }
			}
			else state.EX.nop = true;
            cout << "cycle " << cycle << "\toperation : " << operation << endl;
			// EX
			if (!state.EX.nop) {
				executeInstruction(state.EX, state.MEM, myRF, operation, state);
			}
			else state.MEM.nop = true;

			// MEM
			if (!state.MEM.nop){
				if (state.MEM.rd_mem){
					// cout << operation << " is now reading memory" << endl;
					state.WB.Wrt_data = ext_dmem.readDataMem(state.MEM.ALUresult);
				}
				else {
					if (operation == "JAL") state.WB.Wrt_data = state.MEM.Store_data;
					else {
						state.WB.Wrt_data = state.MEM.ALUresult;
					}
				}
				if (state.MEM.wrt_mem){
					// cout << state.MEM.Store_data << " is writing to memory location "<< state.MEM.ALUresult << endl;
					ext_dmem.writeDataMem(state.MEM.ALUresult, state.MEM.Store_data);
				}
				state.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;
				state.WB.wrt_enable = state.MEM.wrt_enable;
			}
			else state.WB.nop = true;

			// WB
			if (!state.WB.nop && state.WB.wrt_enable) {
				// cout << "Writing " << state.WB.Wrt_data << " to register " << state.WB.Wrt_reg_addr << endl;
				myRF.writeRF(state.WB.Wrt_reg_addr, state.WB.Wrt_data);
			}

			if (state.ID.Instr.to_ulong() == 0xFFFFFFFF){
				state.IF.nop = state.ID.nop = state.EX.nop = state.MEM.nop = state.WB.nop = halted = true;
			}
			else if (operation == "BEQ" || operation == "BNE" || operation == "JAL"){
				int offset = state.MEM.ALUresult.to_ulong();
				if (state.MEM.ALUresult[31] == 1) offset = -TwosComplement(state.MEM.ALUresult).to_ulong();
				// cout << "current address (" << state.IF.PC << ") + offset(" << offset << ")" << endl; 
				state.IF.PC = bitset<32>(state.IF.PC.to_ulong() + offset);
                cout << "offset = " << offset << endl;
                if (operation == "BEQ" || operation == "BNE"){
                    if (offset != 4){
                        myRF.outputRF(cycle, outDir+"/FS"); // dump RF
                        printState(state, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ... 
                        cycle++;
                    }
                    else {
                        // set next state
                        nextState.IF = state.IF;
                        nextState.ID = state.ID;
                        nextState.EX = state.EX;
                        nextState.MEM = state.MEM;
                        nextState.WB = state.WB;

                        clearControlSignals(nextState);
                        
                        state = nextState; 
                        return ;
                    }
                }
			}
			else {
				state.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4);
			}
			
			myRF.outputRF(cycle, outDir+"/FS"); // dump RF
			printState(state, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ... 

			// set next state
			nextState.IF = state.IF;
			nextState.ID = state.ID;
			nextState.EX = state.EX;
			nextState.MEM = state.MEM;
			nextState.WB = state.WB;

			clearControlSignals(nextState);
			
			state = nextState; // The end of the cycle and updates the current state with the values calculated in this cycle
			cycle++;
		}

		void printState(stateStruct state, int cycle) {
			string out(outDir + "/StateResult_FS.txt");
    		ofstream printstate;
			if (cycle == 0)
				printstate.open(out, std::ios_base::trunc);
			else 
    			printstate.open(out, std::ios_base::app);
            if (printstate.is_open()) {
				printstate << "----------------------------------------------------------------------" << endl;
		        printstate<<"State after executing cycle: "<<cycle<<endl; 

				printstate<<"IF.nop: "<<(state.IF.nop ? "True" : "False")<<endl; 
		        printstate<<"IF.PC: "<<state.IF.PC.to_ulong()<<endl;        

				printstate<<"ID.nop: "<<(state.ID.nop ? "True" : "False")<<endl;
		        printstate<<"ID.Instr: "<<state.ID.Instr<<endl; 

				printstate<<"EX.nop: "<<(state.EX.nop ? "True" : "False")<<endl; 
		        printstate<<"EX.Read_data1: "<<state.EX.Read_data1<<endl;
		        printstate<<"EX.Read_data2: "<<state.EX.Read_data2<<endl;
		        printstate<<"EX.Imm: "<<state.EX.Imm<<endl; 
		        printstate<<"EX.Rs: "<<state.EX.Rs<<endl;
		        printstate<<"EX.Rt: "<<state.EX.Rt<<endl;
		        printstate<<"EX.Wrt_reg_addr: "<<state.EX.Wrt_reg_addr<<endl;
		        printstate<<"EX.is_I_type: "<<state.EX.is_I_type<<endl; 
		        printstate<<"EX.rd_mem: "<<state.EX.rd_mem<<endl;
		        printstate<<"EX.wrt_mem: "<<state.EX.wrt_mem<<endl;        
		        printstate<<"EX.alu_op: "<<state.EX.alu_op<<endl;
		        printstate<<"EX.wrt_enable: "<<state.EX.wrt_enable<<endl;

				printstate<<"MEM.nop: "<<(state.MEM.nop ? "True" : "False")<<endl;   
		        printstate<<"MEM.ALUresult: "<<state.MEM.ALUresult<<endl;
		        printstate<<"MEM.Store_data: "<<state.MEM.Store_data<<endl; 
		        printstate<<"MEM.Rs: "<<state.MEM.Rs<<endl;
		        printstate<<"MEM.Rt: "<<state.MEM.Rt<<endl;   
		        printstate<<"MEM.Wrt_reg_addr: "<<state.MEM.Wrt_reg_addr<<endl;              
		        printstate<<"MEM.rd_mem: "<<state.MEM.rd_mem<<endl;
		        printstate<<"MEM.wrt_mem: "<<state.MEM.wrt_mem<<endl; 
		        printstate<<"MEM.wrt_enable: "<<state.MEM.wrt_enable<<endl;         

				printstate<<"WB.nop: "<<(state.WB.nop ? "True" : "False")<<endl; 
		        printstate<<"WB.Wrt_data: "<<state.WB.Wrt_data<<endl;
		        printstate<<"WB.Rs: "<<state.WB.Rs<<endl;
		        printstate<<"WB.Rt: "<<state.WB.Rt<<endl;
		        printstate<<"WB.Wrt_reg_addr: "<<state.WB.Wrt_reg_addr<<endl;
		        printstate<<"WB.wrt_enable: "<<state.WB.wrt_enable<<endl;
		    }
		    else cout<<"Unable to open FS StateResult output file." << endl;
    		printstate.close();
		}
	private:
		string outDir;
        map<int, pair<string, bitset<5>>> prev_op;
};


string decodeInstruction(const bitset<32>& instr, EXStruct &ex, RegisterFile& rf, map<int, pair<string, bitset<5>>>& prev_op, int cycle, string id){
	string instruction = (instr).to_string();
	string opcode = instruction.substr(25, 7);
    string operation = "Unknown instruction";
	// R-TYPE
	if (opcode == "0110011"){
		string funct7 = instruction.substr(0, 7);
		bitset<5> rs2 = bitset<5>(instruction.substr(7, 5));
		bitset<5> rs1 = bitset<5>(instruction.substr(12, 5));
		string funct3 = instruction.substr(17, 3);
		bitset<5> rd = bitset<5>(instruction.substr(20, 5));
		ex.is_I_type = false;
		ex.alu_op = true;
		ex.nop = false;
		ex.rd_mem = false;
		ex.Read_data1 = rf.readRF(rs1);
		ex.Read_data2 = rf.readRF(rs2);
		ex.Rs = rs1;
		ex.Rt = rs2;
		ex.Wrt_reg_addr = rd;
		ex.wrt_enable = true;
		ex.wrt_mem = false;
        if (id == "FS" && hasHazard(prev_op, rs1, rs2, rd)) return "hazard";
		// ADD
		if (funct7 == "0000000" && funct3 == "000") operation = "ADD";
		// SUB
		else if (funct7 == "0100000" && funct3 == "000") operation = "SUB";
		// XOR
		else if (funct7 == "0000000" && funct3 == "100") operation = "XOR";
		// OR
		else if (funct7 == "0000000" && funct3 == "110") operation = "OR";
		// AND
		else if (funct7 == "0000000" && funct3 == "111") operation = "AND";
        prev_op[cycle] = {operation, rd};
	}
	// I-TYPE
	else if (opcode == "0010011"){
		bitset<12> imm(instruction.substr(0, 12));
		bitset<5> rs1(instruction.substr(12, 5));
		string funct3 = instruction.substr(17, 3);
		bitset<5> rd(instruction.substr(20, 5));
		ex.is_I_type = true;
		ex.alu_op = true;
		ex.wrt_enable = true;
		ex.Read_data1 = rf.readRF(rs1);
		ex.Wrt_reg_addr = rd;
		cout << "data1 = " << ex.Read_data1 << endl;
		int imm_signed = imm.to_ulong();
		if (imm[11] == 1){
			imm_signed |= 0xFFFFF000;
		}
		ex.Imm = bitset<16>(imm_signed);
        if (id == "FS" && hasHazard(prev_op, rs1, rs1, rd)) return "hazard";
		// ADDI
		if (funct3 == "000") operation = "ADDI";
		// XORI
		else if (funct3 == "100") operation = "XORI";
		// ORI
		else if (funct3 == "110") operation = "ORI";
		// ANDI
		else if (funct3 == "111") operation = "ANDI";
        prev_op[cycle] = {operation, rd};
	}
	// LW
	else if (opcode == "0000011"){
		bitset<5> rs1 = bitset<5>(instruction.substr(12, 5));
		bitset<5> rd = bitset<5>(instruction.substr(20, 5));
		bitset<12> imm = bitset<12>(instruction.substr(0,12));
		int imm_signed = imm.to_ulong();
		if (imm[11] == 1){
			imm_signed |= 0xFFFFF000;
		}

		// SET CONTROL SIGNALS
		ex.is_I_type = true;
		ex.rd_mem = true;
		ex.wrt_mem = false;
		ex.alu_op = true;
		ex.wrt_enable = true;

		// SET OPERANDS
		ex.Read_data1 = rf.readRF(rs1);
		ex.Imm = bitset<16>(imm_signed);
		ex.Wrt_reg_addr = rd;
		operation = "LW";
        prev_op[cycle] = {operation, rd};
	}
	// S-TYPE
	else if (opcode == "0100011"){
		bitset<12> imm(instruction.substr(0, 7) + instruction.substr(20, 5));
		bitset<5> rs2(instruction.substr(7, 5));
		bitset<5> rs1(instruction.substr(12, 5));
		int imm_signed = imm.to_ulong();
		if (imm[11] == 1){
			imm_signed |= 0xFFFFF000;
		}

		ex.is_I_type = false;
		ex.rd_mem = true;
		ex.wrt_mem = true;
		ex.alu_op = true;
		ex.Imm = bitset<16>(imm_signed);
		ex.Read_data1 = rf.readRF(rs1);
		ex.Read_data2 = rf.readRF(rs2);
		ex.nop = false;
		ex.wrt_enable = false;
		operation = "SW";
        prev_op[cycle] = {operation, bitset<5>(11111)};
	}
	// J-TYPE
	else if (opcode == "1101111"){
		bitset<20> imm(instruction.substr(0,1) + instruction.substr(12,8) + instruction.substr(11,1) + instruction.substr(1,10));
		bitset<5> rd(instruction.substr(20, 5));
		int imm_signed = imm.to_ulong();
		if (imm[19] == 1){
			imm_signed |= 0xFFFFF000;
		}
		ex.is_I_type = false;
		ex.rd_mem = false;
		ex.wrt_mem = false;
		ex.wrt_enable = true;
		ex.Wrt_reg_addr = rd;
		ex.Imm = bitset<16>(imm_signed);
		operation = "JAL";
	}
	// B-TYPE
	else if (opcode == "1100011"){
		bitset<12> imm(instruction.substr(0,1) + instruction.substr(24,1) + instruction.substr(1,6) + instruction.substr(20,4));
		bitset<5> rs2(instruction.substr(7, 5));
		bitset<5> rs1(instruction.substr(12, 5));
		string funct3 = instruction.substr(17, 3);
		ex.is_I_type = false;
		ex.alu_op = true;
		ex.Read_data1 = rf.readRF(rs1);
		ex.Read_data2 = rf.readRF(rs2);

		int imm_signed = imm.to_ulong();
		if (imm[11] == 1){
			imm_signed |= 0xFFFFF000;
		}
		ex.Imm = bitset<16>(imm_signed);
		// BEQ
		if (funct3 == "000") operation = "BEQ";
		// BNE
		else if (funct3 == "001") operation = "BNE";
	}
    if (prev_op.size() == 2) prev_op.erase(prev_op.begin());
	return operation;
}

bool hasHazard(map<int, pair<string, bitset<5>>>& prev_op, bitset<5> rs1, bitset<5> rs2, bitset<5> rd){
    for (auto& p : prev_op){
        string op = p.second.first;
        bitset<5> reg = p.second.second;
        if (op == "LW" && (rs1 == reg || rs2 == reg)) {
            prev_op.erase(prev_op.begin());
            return true;
        }
    }
    return false;
}

void executeInstruction(const EXStruct& ex, MEMStruct& mem, RegisterFile& rf, const string& operation, stateStruct& state){
	if (operation == "LW"){
		bitset<32> baseAddress = ex.Read_data1;
		bitset<32> imm = signExtendImm(ex.Imm);
		int imm_value = imm[31] == 1 ? -(TwosComplement(imm)).to_ulong() : imm.to_ulong();
		bitset<32> final_address = bitset<32>(baseAddress.to_ulong() + imm_value);

		mem.ALUresult = final_address;
		mem.Wrt_reg_addr = ex.Wrt_reg_addr;	// write back
		mem.rd_mem = ex.rd_mem;	// read memory
		mem.wrt_enable = ex.wrt_enable;
		mem.wrt_mem = ex.wrt_mem;
	}
	else if (operation == "SW"){
		bitset<32> imm = signExtendImm(ex.Imm);
		int imm_value = imm[31] == 1 ? -(TwosComplement(imm)).to_ulong() : imm.to_ulong();
		bitset<32> address = bitset<32>((ex.Read_data1).to_ulong() + imm_value);
		mem.ALUresult = address;
		mem.Store_data = ex.Read_data2;
		mem.wrt_mem = ex.wrt_mem;
		mem.wrt_enable = ex.wrt_enable;
	}
	else if (operation == "ADD" || operation == "SUB" || operation == "XOR" || operation == "OR" || operation == "AND" || 
			 operation == "ADDI" || operation == "XORI" || operation == "ORI" || operation == "ANDI"){
		int value1 = (ex.Read_data1[31] == 1 ? -(TwosComplement(ex.Read_data1)).to_ulong() : ex.Read_data1.to_ulong());
		bitset<32> sign_extended = signExtendImm(ex.Imm);
		if (operation == "ADD" || operation == "SUB") {
			int value2 = (ex.Read_data2[31] == 1 ? -(TwosComplement(ex.Read_data2)).to_ulong() : ex.Read_data2.to_ulong());
			if (operation == "ADD") mem.ALUresult = bitset<32>(value1+value2);
			else if (operation == "SUB") mem.ALUresult = bitset<32>(value1 - value2);
		}
		else if (operation == "XOR") mem.ALUresult = bitset<32>(ex.Read_data1 ^ ex.Read_data2);
		else if (operation == "OR") mem.ALUresult = bitset<32>(ex.Read_data1 | ex.Read_data2);
		else if (operation == "AND") mem.ALUresult = bitset<32>(ex.Read_data1 & ex.Read_data2);
		else if (operation == "ADDI" || operation == "XORI" || operation == "ORI" || operation == "ANDI") {
			int imm = sign_extended[31] == 1 ? -(TwosComplement(sign_extended)).to_ulong() : sign_extended.to_ulong();
			if (operation == "ADDI") mem.ALUresult = bitset<32>(value1 + imm);
			else if (operation == "XORI") mem.ALUresult = bitset<32>(ex.Read_data1 ^ signExtendImm(ex.Imm));
			else if (operation == "ORI") mem.ALUresult = bitset<32>(ex.Read_data1 | signExtendImm(ex.Imm));
			else if (operation == "ANDI") mem.ALUresult = bitset<32>(ex.Read_data1 & signExtendImm(ex.Imm));
		}
		mem.Wrt_reg_addr = ex.Wrt_reg_addr;
		mem.wrt_enable = ex.wrt_enable;
		mem.rd_mem = ex.rd_mem;
	}
	else if (operation == "JAL"){
		mem.wrt_enable = true;
		mem.Wrt_reg_addr = ex.Wrt_reg_addr;
		mem.Store_data = bitset<32>(state.IF.PC.to_ulong() + 4);
		mem.ALUresult = signExtendImm(ex.Imm) << 1;
	}
	else if (operation == "BNE" || operation == "BEQ"){
		if (operation == "BNE"){
			mem.ALUresult = (ex.Read_data1 == ex.Read_data2 ? 4 : (signExtendImm(ex.Imm) << 1));
		}
		else if (operation == "BEQ"){
			mem.ALUresult = (ex.Read_data1 != ex.Read_data2 ? 4 :  (signExtendImm(ex.Imm) << 1));
		}
	}
}

bitset<32> signExtendImm(const bitset<16>& imm){
	string extended = "";
	if (imm[15] == 1) extended = string(16, '1') + imm.to_string();
	else extended = string(16, '0') + imm.to_string();
	return bitset<32>(extended);
}

void clearControlSignals(stateStruct &nextState) {
    // Reset the control signals for the next cycle
    nextState.ID.nop = nextState.IF.nop = nextState.EX.nop = nextState.MEM.nop = nextState.WB.nop = false;
    nextState.EX.is_I_type = nextState.EX.rd_mem = nextState.EX.wrt_mem = nextState.EX.alu_op = nextState.EX.wrt_enable = false;
    nextState.MEM.rd_mem = nextState.MEM.wrt_mem = nextState.MEM.wrt_enable = false;
    nextState.WB.wrt_enable = false;
}

bitset<32> TwosComplement(bitset<32> num){
	if (num[31] == 1) return bitset<32>((~num).to_ulong() + 1);
	else return num;
}

void outputPerformanceMetrics(int SS_cycle, int SS_num_instr, int FS_cycle, int FS_num_instr, const string& path){
	ofstream printMetrics;
	printMetrics.open(path + "/PerformanceMetrics.txt", std::ios_base::trunc);
	if (printMetrics.is_open()){
		printMetrics << "Performance of Single Stage:" << endl;
		printMetrics << "#Cycles -> " << SS_cycle << endl;
		printMetrics << "#Instructions -> " << SS_num_instr << endl;
		printMetrics << "CPI -> " << (double)SS_cycle/SS_num_instr << endl;
		printMetrics << "IPC -> " << (double)SS_num_instr/SS_cycle << endl << endl;
        printMetrics << "Performance of Five Stage:" << endl;
        printMetrics << "#Cycles -> " << FS_cycle << endl;
        printMetrics << "#Instructions -> " << FS_num_instr << endl;
        printMetrics << "CPI -> " << (double)FS_cycle/FS_num_instr << endl;
		printMetrics << "IPC -> " << (double)FS_num_instr/FS_cycle << endl;
	}
	else cout << "Unable to open PerformanceMetrics.txt output file." << endl;
	printMetrics.close();
}

int main(int argc, char* argv[]) {
	
	string ioDir = "";
    if (argc == 1) {
        cout << "Enter path containing the memory files: ";
        cin >> ioDir;
    }
    else if (argc > 2) {
        cout << "Invalid number of arguments. Machine stopped." << endl;
        return -1;
    }
    else {
        ioDir = argv[1];
        cout << "IO Directory: " << ioDir << endl;
    }

	// Create the output_netID folder in the parent folder
	fs::path input_path = ioDir;
	fs::path output_netID = input_path.parent_path().parent_path() / "output_hz3478";
    try {
        if (fs::create_directories(output_netID)) {  // create_directories also creates all intermediate directories if needed
            std::cout << "Directory created successfully: " << output_netID << std::endl;
        } else {
            std::cout << "Directory was not created (it may already exist): " << output_netID << std::endl;
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << e.what() << std::endl;
    }

	// Create testcase- folder in output_netID folder
	std::string folder_name = input_path.filename().string();
	fs::path output_path = input_path.parent_path().parent_path() / "output_hz3478" / folder_name;
    try {
        if (fs::create_directories(output_path)) {  // create_directories also creates all intermediate directories if needed
            std::cout << "Directory created successfully: " << output_path << std::endl;
        } else {
            std::cout << "Directory was not created (it may already exist): " << output_path << std::endl;
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << e.what() << std::endl;
    }

    InsMem imem = InsMem("Imem", ioDir);
    DataMem dmem_ss = DataMem("SS", ioDir);
    DataMem dmem_fs = DataMem("FS", ioDir);
	// DataMem dmem_fs = DataMem("FS", ioDir);

    SingleStageCore SSCore(ioDir, output_path.string(), imem, dmem_ss);
	FiveStageCore FSCore(ioDir, output_path.string(), imem, dmem_fs);
	// FiveStageCore FSCore(ioDir, imem, dmem_fs);

    // Start single stage pipeline
    while (1) {
		if (!SSCore.halted)
			SSCore.step(dmem_ss);
		if (SSCore.halted) {
			SSCore.step(dmem_ss);
			break;
		}
    }

    // State five stage pipeline
    while (1) {
		if (!FSCore.halted)
			FSCore.step(dmem_fs);
		if (FSCore.halted) {
			FSCore.step(dmem_fs);
			break;
		}
    }
    
	// // dump SS and FS data mem.
    dmem_ss.outputDataMem(output_path.string());
	dmem_fs.outputDataMem(output_path.string());
	outputPerformanceMetrics(SSCore.cycle+1, SSCore.cycle, FSCore.cycle, SSCore.cycle, output_path.string());

	return 0;
}