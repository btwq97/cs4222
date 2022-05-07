In order to compile and use our program, follow the steps below:

1) Proceed to the "~/contiki/examples/nbr_discovery" folder.
2) Place the downloaded nbr_discovery.c file in the above folder.
3) Open a terminal in this folder and use the following command in terminal to compile the program. "make TARGET=srf06-cc26xx BOARD=sensortag/cc2650 nbr_discovery.bin CPU_FAMILY=cc26xx".
4) Using UniFlash, upload the compiled nbr_discovery.bin file to the SensorTag - CC2650.
5) After the program has been uploaded to the SensorTags, reboot the SensorTag.



In order to use the program in Cooja Simulator, follow the steps below:

1) Proceed to the "~/contiki/examples/nbr_discovery" folder.
2) Place the downloaded nbr_discovery.c file in the above folder.
3) Open a terminal in this folder and use the following command in terminal to compile the program. "make TARGET=sky nbr_discovery.upload".
4) Upon compilation, run Cooja using the "ant run" command in the "~/contiki/tools/cooja" directory.
5) In Cooja, open the simulator file using "File->Open_simulation" in the GUI.
6) Select the "nbr_discovery/cooja_nbr_discovery.csc" file.
7) Run the simulation.


