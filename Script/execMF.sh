#!/bin/bash

BASE_OUTPUT_DIR="results/"
#EXEC="/media/angelo/BigLinux/Programs/Eclipse/EclipseCPP/workspaces/wakeUpDrone/WakeUpDroneSimulator/Release/WakeUpDroneSimulator"
EXEC=$1
B_RUNS=$2
N_RUNS=$3
TEST_NAME=$4

Nuav=2
Nsensors=100
Altitude=5
Alpha=0.75
#TimeExp=604800

StatOnRun=0
StatToLog=3600 #1 ora

varKE=0.005
varME=1800
varKD=0.004
varMD=2500		# 5 Km
varKT=0.0006
varMT=15000		# 12 H

useSigmoid=1
fullEnergy=1

# ./WakeUpDroneSimulator -algoClust rrMinLoss -algoTSP tsp2optE -nu 1 -ns 10 -time -1 -statFile results/dummytest/test.log -statOnrun 1 -stat2l 100000 -hmFile results/dummytest/hitmap.log

#for Alpha in 0 0.25 0.5 0.75 1
#for TimeExp in 86400 2000000000
for TimeExp in 86400
do
	echo "SimTime: ${SimTime}"

	#for Nuav in 1 2 4 8 12
	for Nsensors in {20..100..20}
	#for Nsensors in 80
	#for Nsensors in {60..80..20}
	do
		#echo "Number of UAV: ${Nuav}"
		echo "Number of sensors: ${Nsensors}"
		
		#for Nsensors in {50..250..50}
		#for Nuav in 1 2 4 6 8
		for Nuav in 6
		do
			echo "Number of UAV: ${Nuav}"
			#echo "Number of sensors: ${Nsensors}"
			
			for AlgoMain in "treemultiflow" "treemultiflowdistr"
			do
				echo "Algorithm: ${AlgoMain}"
									
				MaxLoss=0.99
				for AlgoType in "bsfdist" "bsfenergy" "dsfdist" "dsfenergy" "bsf" "dsf"
				do		
					for UsePOT in 0 1
					do							
						for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
						do
							StatFolder="${BASE_OUTPUT_DIR}${TEST_NAME}/${AlgoMain}"
							mkdir -p ${StatFolder}
							#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_ML1_R${runs}.log"
							StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}.log"
							LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}.log"
							HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}"
							
							STR_EXEC="$EXEC -seed ${runs} -sFE ${fullEnergy} -st ${AlgoMain} -at ${AlgoType} -mfUsePOT ${UsePOT} -mfMaxLoss ${MaxLoss} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
								
							echo "Executing: '${STR_EXEC}'"
							echo -n "Run: ${runs} starting at "
							date

							if [ -f "${HitFile}.log" ]
							then
								echo "Simulation Done."
							else
								${STR_EXEC} &>${LogFile}
							fi																
						done
					done	
				done
			done		
		done
	done
done





for TimeExp in 86400
do
	echo "SimTime: ${SimTime}"

	#for Nuav in 1 2 4 8 12
	for Nsensors in 80
	#for Nsensors in 80
	#for Nsensors in {60..80..20}
	do
		#echo "Number of UAV: ${Nuav}"
		echo "Number of sensors: ${Nsensors}"
		
		#for Nsensors in {50..250..50}
		for Nuav in 1 2 4 6 8 10
		#for Nuav in 6
		do
			echo "Number of UAV: ${Nuav}"
			#echo "Number of sensors: ${Nsensors}"
			
			for AlgoMain in "treemultiflow" "treemultiflowdistr"
			do
				echo "Algorithm: ${AlgoMain}"
									
				MaxLoss=0.99
				for AlgoType in "bsfdist" "bsfenergy" "dsfdist" "dsfenergy" "bsf" "dsf"
				do		
					for UsePOT in 0 1
					do										
						for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
						do
							StatFolder="${BASE_OUTPUT_DIR}${TEST_NAME}/${AlgoMain}"
							mkdir -p ${StatFolder}
							#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_ML1_R${runs}.log"
							StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}.log"
							LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}.log"
							HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}"
							
							STR_EXEC="$EXEC -seed ${runs} -sFE ${fullEnergy} -st ${AlgoMain} -at ${AlgoType} -mfUsePOT ${UsePOT} -mfMaxLoss ${MaxLoss} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
								
							echo "Executing: '${STR_EXEC}'"
							echo -n "Run: ${runs} starting at "
							date

							if [ -f "${HitFile}.log" ]
							then
								echo "Simulation Done."
							else
								${STR_EXEC} &>${LogFile}
							fi																
						done
					done
				done	
			done		
		done
	done
done




dischargeRate=60

for TimeExp in 2000000000
do
	echo "SimTime: ${SimTime}"

	#for Nuav in 1 2 4 8 12
	for Nsensors in {20..100..20}
	#for Nsensors in 80
	#for Nsensors in {60..80..20}
	do
		#echo "Number of UAV: ${Nuav}"
		echo "Number of sensors: ${Nsensors}"
		
		#for Nsensors in {50..250..50}
		#for Nuav in 1 2 4 6 8
		for Nuav in 6
		do
			echo "Number of UAV: ${Nuav}"
			#echo "Number of sensors: ${Nsensors}"
			
			for AlgoMain in "treemultiflow" "treemultiflowdistr"
			do
				echo "Algorithm: ${AlgoMain}"
									
				MaxLoss=0.99
				for AlgoType in "bsfdist" "bsfenergy" "dsfdist" "dsfenergy" "bsf" "dsf"
				do		
					#for UsePOT in 0 1
					for UsePOT in 1
					do									
						for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
						do
							StatFolder="${BASE_OUTPUT_DIR}${TEST_NAME}/${AlgoMain}"
							mkdir -p ${StatFolder}
							#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_ML1_R${runs}.log"
							StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}.log"
							LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}.log"
							HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}"
							
							STR_EXEC="$EXEC -seed ${runs} -ssd ${dischargeRate} -sFE ${fullEnergy} -st ${AlgoMain} -at ${AlgoType} -mfUsePOT ${UsePOT} -mfMaxLoss ${MaxLoss} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
								
							echo "Executing: '${STR_EXEC}'"
							echo -n "Run: ${runs} starting at "
							date

							if [ -f "${HitFile}.log" ]
							then
								echo "Simulation Done."
							else
								${STR_EXEC} &>${LogFile}
							fi																
						done
					done
				done	
			done		
		done
	done
done





for TimeExp in 2000000000
do
	echo "SimTime: ${SimTime}"

	#for Nuav in 1 2 4 8 12
	for Nsensors in 80
	#for Nsensors in 80
	#for Nsensors in {60..80..20}
	do
		#echo "Number of UAV: ${Nuav}"
		echo "Number of sensors: ${Nsensors}"
		
		#for Nsensors in {50..250..50}
		for Nuav in 1 2 4 6 8 10
		#for Nuav in 6
		do
			echo "Number of UAV: ${Nuav}"
			#echo "Number of sensors: ${Nsensors}"
			
			for AlgoMain in "treemultiflow" "treemultiflowdistr"
			do
				echo "Algorithm: ${AlgoMain}"
									
				MaxLoss=0.99
				for AlgoType in "bsfdist" "bsfenergy" "dsfdist" "dsfenergy" "bsf" "dsf"
				do		
					#for UsePOT in 0 1
					for UsePOT in 1
					do									
						for (( runs=B_RUNS; runs<=N_RUNS; runs++ ))
						do
							StatFolder="${BASE_OUTPUT_DIR}${TEST_NAME}/${AlgoMain}"
							mkdir -p ${StatFolder}
							#StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_A${Alpha}_ML1_R${runs}.log"
							StatFile="${StatFolder}/stat_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}.log"
							LogFile="${StatFolder}/log_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}.log"
							HitFile="${StatFolder}/hit_NU${Nuav}_NS${Nsensors}_AT${AlgoType}_T${TimeExp}_UP${UsePOT}_R${runs}"
							
							STR_EXEC="$EXEC -seed ${runs} -ssd ${dischargeRate} -sFE ${fullEnergy} -st ${AlgoMain} -at ${AlgoType} -mfUsePOT ${UsePOT} -mfMaxLoss ${MaxLoss} -nu ${Nuav} -ns ${Nsensors} -sFR 1 -alpha ${Alpha} -time ${TimeExp} -statFile ${StatFile} -statOnrun ${StatOnRun} -stat2l ${StatToLog} -hmFile ${HitFile} -ke ${varKE} -me ${varME} -kd ${varKD} -md ${varMD} -kt ${varKT} -mt ${varMT} -uSigm ${useSigmoid}"
								
							echo "Executing: '${STR_EXEC}'"
							echo -n "Run: ${runs} starting at "
							date

							if [ -f "${HitFile}.log" ]
							then
								echo "Simulation Done."
							else
								${STR_EXEC} &>${LogFile}
							fi																
						done
					done
				done	
			done		
		done
	done
done




























