The README file of paper **"Efficient 3D placement of drone base stations with frequency planning"**.

### How to build and execute ? ###
The case generator is located in **gen** subdir. To compile it, just type
> make

Then, binary executable file **genCases** is produced, its usage:
> ./genCases -x X_coord -y Y_coord -n user_num -k UAV_num -s seed case_file.txt

The GUI version located in current dir depends on Qt4. If you have not installed Qt4 or qmake, it can not be compiled. If you use Qt Creator as IDE, binary executable file is generated in dir ../build-UAV-Desktop-Debug.

The command version has no library dependent, and it is located in **cmd** subdir. To compile it, just type
> make

Then, binary executable file **UAV** is produced, its usage:
> ./UAV case_file.txt

### How to do batch jobs ? ###
First, in the dir of this **README.md** file, type
> mkdir cases

In **gen** subdir, file **batchGen.sh** is used to generate a large number of cases. Before use it, ensure it is executable, if not, just type
> chmod u+x batchGen.sh

its usage:
> ./batchGen.sh X_coord Y_coord user_num UAV_num

Afterwords, case files are generated in **cases** subdir.

In **cmd** subdir, file **batchExec.sh** is used to execute a large number of cases and record the statistics of each case. Similarly, before use it, ensure it is executable. Its usage:
> ./batchExec.sh path/to/case/file/dir

Afterwords, file **statistic.csv** is generated in the case dir you specify, each row of this file corresponding to one case. To calculate the average statistics of all cases of the same type, use tool awk as follows:
> awk -F ',' -f calcAverage.awk path/to/the/file/statistic.csv

### How to plot solutions ? ###
File **draw.m** plots solutions in Matlab. Three files **user.csv, UAVs.csv, servedUsers.csv** are needed in plotting for each case, and they are located in the case dir you specify when you do batch jobs.
