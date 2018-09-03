#pragma once

#ifdef __cplusplus
 extern "C" {
#endif 
     
typedef void (*shell_cmd)(int argc, char *argv[]);      
     
void cli_init(void);     
void cli_updata(void);
void cli_regist(const char* name, shell_cmd cmd);

     
     
#ifdef __cplusplus
}
#endif

