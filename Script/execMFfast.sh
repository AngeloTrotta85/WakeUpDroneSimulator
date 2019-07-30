#!/bin/bash

BASE_OUTPUT_DIR="results/"
EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/wakeUpDrone/WakeUpDroneSimulator/Release/WakeUpDroneSimulator"
B_RUNS=$1
N_RUNS=$2

Nuav=2
Nsensors=100
Altitude=5
Alpha=0.75
#TimeExp=604800

StatOnRun=0
StatToLog=3600 #1 ora

varKE=0.005
varME=1800
varKD=0.005
varMD=2000		# 5 Km
varKT=0.0006
varMT=15000		# 12 H

useSigmoid=1

# ./WakeUpDroneSimulator -algoClust rrMinLoss -algoTSP tsp2optE -nu 1 -ns 10 -time -1 -statFile results/dummytest/test.log -statOnrun 1 -stat2l 100000 -hmFile results/dummytest/hitmap.log

#for Alpha in 0 0.25 0.5 0.75 1
for TimeExp in 86400
do
	echo "SimTime: ${SimTime}"

	#for Nuav in 1 2 4 8 12
	for Nsensors in {20..60..20}
	#for Nsensors in 80
	#for Nsensors in {60..80..20}
	do
		#echo "Number of UAV: ${Nuav}"
		echo "Number of sensors: ${Nsensors}"
		
		#for Nsensors in {50..250..50}
		for Nuav in 1 2 4 8
		#for Nuav in 4 6
		do
			echo "Number of UAV: ${Nuav}"
			#echo "Number of sensors: ${Nsensors}"
			
			for AlgoMain in "treemultiflow" "treemultiflowdistr"
			do
				echo "Algorithm: ${AlgoMain}"
				
				for AlgoType in "bsf" "dsf"
				do			
					for MaxLoss in 1 0.8 0.6
					do										
						for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
						do
							StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
							mkdir -p ${StatFolder}
							#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
							StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}.log"
							LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_R${runs}.log"
							HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_R${runs}"
							
							STR_EXEC="$EXEC -seed ${runs} -st ${AlgoMain} -at ${AlgoType} -mfMaxLoss ${MaxLoss} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
								
							echo "Executing: '${STR_EXEC}'"
							echo -n "Run: ${runs} starting at "
							date
							
							if [ -f "$StatFile" ]
							then
								SEARCHRUN=`cat ${StatFile} | grep "^${runs};"`
								if [ -z "$SEARCHRUN" ]
								then
								      echo "Simulation Done."
								else
								      ${STR_EXEC} &>${LogFile}
								fi
								#echo "Simulation Done."
							else
								${STR_EXEC} &>${LogFile}
							fi											
						done
					done
				done	
				
				for AlgoType in "bsfdist" "bsfenergy" "dsfdist" "dsfenergy"
				do									
					for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
					do
						StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
						mkdir -p ${StatFolder}
						#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}.log"
						LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_R${runs}.log"
						HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_R${runs}"
						
						STR_EXEC="$EXEC -seed ${runs} -st ${AlgoMain} -at ${AlgoType} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
							
						echo "Executing: '${STR_EXEC}'"
						echo -n "Run: ${runs} starting at "
						date
						
						if [ -f "$StatFile" ]
						then
							SEARCHRUN=`cat ${StatFile} | grep "^${runs};"`
							if [ -z "$SEARCHRUN" ]
							then
							      echo "Simulation Done."
							else
							      ${STR_EXEC} &>${LogFile}
							fi
							#echo "Simulation Done."
						else
							${STR_EXEC} &>${LogFile}
						fi											
					done
				done	
			done		
		done
	done
done
