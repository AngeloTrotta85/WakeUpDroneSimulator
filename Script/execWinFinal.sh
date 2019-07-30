#!/bin/bash

BASE_OUTPUT_DIR="results/"
EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/wakeUpDrone/WakeUpDroneSimulator/Release/WakeUpDroneSimulator"
B_RUNS=$1
N_RUNS=$2

Nuav=2
Nsensors=100
Altitude=5
Alpha=0.75
TimeExp=604800

AlgoClust=rrMinLoss
AlgoTSP=tsp2optE

StatOnRun=1
StatToLog=3600 #1 ora

varKE=0.005
varME=1800
varKD=0.002
varMD=5000
varKT=0.0006
#varMT=43200
varMT=45000
useSigmoid=1

# ./WakeUpDroneSimulator -algoClust rrMinLoss -algoTSP tsp2optE -nu 1 -ns 10 -time -1 -statFile results/dummytest/test.log -statOnrun 1 -stat2l 100000 -hmFile results/dummytest/hitmap.log


#for Nuav in 1 2 4 8 12
#for Nsensors in {20..100..20}
for Nsensors in 80
#for Nsensors in {60..80..20}
do
	#echo "Number of UAV: ${Nuav}"
	echo "Number of sensors: ${Nsensors}"
	
	#for Nsensors in {50..250..50}
	for Nuav in 1 2 4 6 8 10 12
	#for Nuav in 4 6
	do
		echo "Number of UAV: ${Nuav}"
		#echo "Number of sensors: ${Nsensors}"
		
		for AlgoMain in "bee" "beenc" "close" "lowbatt"
		#for AlgoMain in "close" "lowbatt"
		do
			echo "Algorithm: ${AlgoMain}"
			
			if [ "$AlgoMain" == "bee" ] || [ "$AlgoMain" == "beenc" ] ; then
			
				AlgoTSP=tsp2optE					
				
				#for Alpha in 0 0.25 0.5 0.75 1
				for Alpha in 0.75
				do
					echo "Alpha: ${Alpha}"
					for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
					do
						
						StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
						mkdir -p ${StatFolder}
						StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
								
						STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
					
						
						echo "Executing: '${STR_EXEC}'"
						echo -n "Run: ${runs} starting at "
						date
							
						${STR_EXEC} &>${LogFile}	
					done
				done
			else
				Alpha=0.75
				if [ "$AlgoMain" == "close" ]; then
					AlgoTSP=tsp2optEMinDist
				else
					AlgoTSP=tsp2optEMinEnergy
				fi
				
				for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
				do
					
					StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
					mkdir -p ${StatFolder}
					StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
				
					STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"				
					
					echo "Executing: '${STR_EXEC}'"
					echo -n "Run: ${runs} starting at "
					date
						
					${STR_EXEC} &>${LogFile}	
				done
			fi
		done
	done
done























#for Nuav in 1 2 4 8 12
for Nsensors in {20..120..20}
#for Nsensors in 80
#for Nsensors in {60..120..20}
do
	#echo "Number of UAV: ${Nuav}"
	echo "Number of sensors: ${Nsensors}"
	
	#for Nsensors in {50..250..50}
	#for Nuav in 1 2 4 6 8 10 12
	for Nuav in 4
	do
		echo "Number of UAV: ${Nuav}"
		#echo "Number of sensors: ${Nsensors}"
		
		for AlgoMain in "bee" "beenc" "close" "lowbatt"
		#for AlgoMain in "close" "lowbatt"
		do
			echo "Algorithm: ${AlgoMain}"
			
			if [ "$AlgoMain" == "bee" ] || [ "$AlgoMain" == "beenc" ] ; then
			
				AlgoTSP=tsp2optE					
				
				#for Alpha in 0 0.25 0.5 0.75 1
				for Alpha in 0.75
				do
					echo "Alpha: ${Alpha}"
					for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
					do
						
						StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
						mkdir -p ${StatFolder}
						StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						
						#STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -nu ${Nuav} -ns ${Nsensors} -time ${TimeExp} -sFR 1 -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
						
						#STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
						STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
					
						
						echo "Executing: '${STR_EXEC}'"
						echo -n "Run: ${runs} starting at "
						date
							
						${STR_EXEC} &>${LogFile}	
					done
				done
			else
				Alpha=0.75
				if [ "$AlgoMain" == "close" ]; then
					AlgoTSP=tsp2optEMinDist
				else
					AlgoTSP=tsp2optEMinEnergy
				fi
				
				for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
				do
					
					StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
					mkdir -p ${StatFolder}
					StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
				
					STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"				
					
					echo "Executing: '${STR_EXEC}'"
					echo -n "Run: ${runs} starting at "
					date
						
					${STR_EXEC} &>${LogFile}	
				done
			fi
		done
	done
done




#for Nuav in 1 2 4 8 12
#for Nsensors in {20..100..20}
for Nsensors in 80
#for Nsensors in {60..120..20}
do
	#echo "Number of UAV: ${Nuav}"
	echo "Number of sensors: ${Nsensors}"
	
	#for Nsensors in {50..250..50}
	#for Nuav in 1 2 4 6 8 10 12
	for Nuav in 4
	do
		echo "Number of UAV: ${Nuav}"
		#echo "Number of sensors: ${Nsensors}"
		
		for AlgoMain in "bee" "beenc"
		#for AlgoMain in "close" "lowbatt"
		do
			echo "Algorithm: ${AlgoMain}"
			
			if [ "$AlgoMain" == "bee" ] || [ "$AlgoMain" == "beenc" ] ; then
			
				AlgoTSP=tsp2optE					
				
				#for Alpha in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1
				for Alpha in 0 0.25 0.5 1
				do
					echo "Alpha: ${Alpha}"
					for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
					do
						
						StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
						mkdir -p ${StatFolder}
						StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
						HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
				
						STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
					
						
						echo "Executing: '${STR_EXEC}'"
						echo -n "Run: ${runs} starting at "
						date
							
						${STR_EXEC} &>${LogFile}	
					done
				done
			else
				Alpha=0.5
				if [ "$AlgoMain" == "close" ]; then
					AlgoTSP=tsp2optEMinDist
				else
					AlgoTSP=tsp2optEMinEnergy
				fi
				
				for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
				do
					
					StatFolder="${BASE_OUTPUT_DIR}${3}/${AlgoMain}"
					mkdir -p ${StatFolder}
					StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
					HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_A${Alpha}_R${runs}.log"
				
					STR_EXEC="$EXEC -seed ${runs} -algoClust ${AlgoClust} -algoTSP ${AlgoTSP} -algoMain ${AlgoMain} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"				
					
					echo "Executing: '${STR_EXEC}'"
					echo -n "Run: ${runs} starting at "
					date
						
					${STR_EXEC} &>${LogFile}	
				done
			fi
		done
	done
done
