Please follow below description for executing examples in the '_internal' folder.

1. Open any _internal/example folder and copy only the source files (header files) into the master application. 
   And please do not copy the 'projects' and 'Makefile' folders present in the example folder into master_application. 

2. Go to 'internal/master_application/projects' folder and access the sample project file.

	Access the below sample projects for executing in baremetal/FreeRTOS environment.
	1.keil_baremetal/sample_project.uvprojx
        2.keil_freertos/sample_project.uvprojx

Note: 
As of now '_internal' examples are verified with 'sample project' by manually copying the source files into master application. This is tedious procedure to execute any example. 
Now all the _internal/examples are provided their own 'projects' folder, consists of Keil project. But this project is only verified with compilation of all the projects but not 
tested the functionality of all the _internal/examples. The next build will be having tested project folder and will avoid the master application/sample project method. 
