# PC-Simulation des Ketten-Updates:  make && ./sim   (./sim v = ausfuehrlich, SIMLOG=1 ./sim = Ablaufprotokoll)
R = ../..
INC = -I. -I$(R)/common -I$(R)/ATTINYBootloader/src -I$(R)/ATTINYCellModule/include -I$(R)/ESPController/include
sim: sim.cpp $(R)/ATTINYBootloader/src/bl_core.c $(R)/ATTINYCellModule/src/fwupdate_module.cpp $(R)/ESPController/src/ModuleFirmwareUpdate.cpp
	gcc -O2 -Wall -Wextra -c $(INC) $(R)/ATTINYBootloader/src/bl_core.c -o bl_core.o
	g++ -O2 -std=c++17 -Wall -Wextra -DFW_HOST_TEST $(INC) sim.cpp $(R)/ATTINYCellModule/src/fwupdate_module.cpp $(R)/ESPController/src/ModuleFirmwareUpdate.cpp bl_core.o -o sim
clean:
	rm -f sim *.o
