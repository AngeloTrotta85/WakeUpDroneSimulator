#!/bin/bash

BASE_OUTPUT_DIR="results/"
EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/wakeUpDrone/WakeUpDroneSimulator/Release/WakeUpDroneSimulator"
B_RUNS=$1
N_RUNS=$2

Nuav=2
Nsensors=100
Altitude=5
Alpha=0.5
TimeExp=604800

AlgoClust=rrMinLoss
AlgoTSP=tsp2optE

StatOnRun=1
StatToLog=3600 #1 ora

# ./WakeUpDroneSimulator -algoClust rrMinLoss -algoTSP tsp2optE -nu 1 -ns 10 -time -1 -statFile results/dummytest/test.log -statOnrun 1 -stat2l 100000 -hmFile results/dummytest/hitmap.log


for Nuav in 1 2 4 8 12
#for Nuav in 4 8 12
do
	echo "Number of UAV: ${Nuav}"
	
	#for Nsensors in {50..200..50}
	for Nsensors in 250
	do
		echo "Number of sensors: ${Nsensors}"
		for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
		do
			
			StatFolder="${BASE_OUTPUT_DIR}${3}"
			mkdir -p ${StatFolder}
			StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
			LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
			HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
			
			STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -nu ${Nuav} -ns ${Nsensors} -time ${TimeExp} -sFR 1 -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile}"
			
			echo "Executing: '${STR_EXEC}'"
			echo -n "Run: ${runs} starting at "
			date
				
			${STR_EXEC} &>${LogFile}	
		done
	done
done
