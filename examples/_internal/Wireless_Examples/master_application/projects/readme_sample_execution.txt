 master_application:
 -------------------
 
 Note:
 =====
 Do not delete projects folder under master_application
 
 ==================================
 Steps for executing STM32 projects: 
 ==================================
 a) Keil_Baremetal - Refer to section 'examples\_internal\Wireless_Examples\master_application\projects'.
 b) Keil_Freertos - Refer to section 'examples\_internal\Wireless_Examples\master_application\projects'. 
 
 To test specific example from _internal,
 ----------------------------------------
 1. Delete existing files (.c and .h) under master_application, copy and paste specific example files(.c and .h) into master_application folder. 
 2. Navigate to projects/(keil_baremetal or keil_freertos) folder in master_application, open sample_project.uvprojx 
 3. Go to keil project workspace and remove the already existing files from example folder
 4. Add specific example files(.c and .h) from the master_application and compile project   
 5. Repeat steps 1 to 4 for compile/build any example with the sample project  
 
 