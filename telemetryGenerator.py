from time import gmtime, strftime, sleep
from balloonScript import TELEMETRY_FILE_BASE


if __name__ == '__main__':
	counter = 0
	outputFileName = "Init_LOG_" + strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".log"
	outputFile = open(outputFileName, "w")
	
	while (True):
		currentTime = strftime("%Y-%m-%d_%H-%M-%S", gmtime())
		if (currentTime[-1] == "0"):
			outputFileName = TELEMETRY_FILE_BASE + "_" + currentTime + ".log"
			outputFile = open(outputFileName, "w")
		outputFile.write(str(counter) + currentTime + "\n")
		counter += 1
		sleep(0.5)