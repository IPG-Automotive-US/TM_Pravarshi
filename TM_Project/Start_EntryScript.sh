#!/bin/bash
BASENAME="${0##*/}"

# Standard function to print an error and exit with a failing return code
error_exit () {
  echo "${BASENAME} - ${1}" >&2
  exit 1
}





# Check that necessary programs are available
which aws >/dev/null 2>&1 || error_exit "Unable to find AWS CLI executable."


#Get the TestRun File from GitHub
ls /home/TM_Project/Data/TestRun/ >>log.txt
LINE=$((AWS_BATCH_JOB_ARRAY_INDEX + 1))
FileName=$(sed -n ${LINE}p log.txt)
echo ${FileName}
export FileName
./src_tm/TruckMaker.linux64 ${FileName} -v -screen -dstore

aws s3 cp /home/CM_Projects/SimOutput/ ${RESULT_FILE_S3_URL} --recursive --exclude "*" --include "*.dat" 
aws s3 cp /home/CM_Projects/SimOutput/ ${RESULT_FILE_S3_URL} --recursive --exclude "*" --include "*.dat.info" 
