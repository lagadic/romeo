#! /bin/sh

for file in *.qc
do
    echo ${file}
    filename=${file%.*}
    #echo ${filename}
    file_out="stpbv_tmp/${filename}.txt"
    if [ -f "$file_out" ]
      then
        echo "$file_out already created. Skip."
      else
        sch_creator ${file} stpbv_tmp/${filename}.txt
    fi
done
