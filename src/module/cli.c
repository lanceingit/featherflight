#include <stdint.h>
#include <string.h>
#include "usbd_cdc_vcp.h"
#include "debug.h"
#include "mm.h"
#include "cli.h"

/*
intput -> string ->  cmd/arg0/arg1/... -> function(arg0,arg1,...) -> output

name -> cmd  == string -> function
string -> arg == string parse

*/



struct buildin_s
{
    const char* name;
    shell_cmd cmd;
};

#define BUFFER_SIZE 50 
static char cmd_buf[BUFFER_SIZE+1];
static char arg_buf[BUFFER_SIZE+1];
static uint8_t read_cnt=0;
static uint16_t cmd_total;

static struct buildin_s shell[100];
static void cli_handle_cmd(char* buf);

void help_shell(int argc, char *argv[]);
void reboot_shell(int argc, char *argv[]);

void cli_init(void)
{
    mm_init((void*)arg_buf, BUFFER_SIZE+1); 
    cli_regist("help", help_shell);
    cli_regist("reboot", reboot_shell);
}

void cli_updata(void)
{
    char c;   
    
    while(!usb_IsEmpty() && read_cnt < BUFFER_SIZE)
    {
        usb_get((uint8_t*)&c);
        DEBUG_PRINTF("%c", c);
        if(c == '\b')
        {
            DEBUG_PRINTF(" \b");
            if(read_cnt > 0)
            {  
                read_cnt--;
            }
        } else if(c == '\n' || c == '\r') {   
            if(c == '\r') DEBUG_PRINTF("\n");
            cmd_buf[read_cnt] = '\0';
            //DEBUG_PRINTF("cmd:%s\r\n", cmd_buf);
            cli_handle_cmd(cmd_buf);
            read_cnt = 0;
            DEBUG_PRINTF("\r>");
        }
        else
        {
            cmd_buf[read_cnt++] = c;
        }
    }    

    if(read_cnt >=  BUFFER_SIZE)
    {   
        read_cnt = 0;
    }     
}


static void cli_handle_cmd(char* buf)
{
    uint8_t len = strlen(buf);
    
    if(len==0||(len==1&&!strcmp(buf, "\n"))||(len==2&&!strcmp(buf, "\r\n")))
    {
        return;
    }
    
    
    
    int argc=0;
    char *argv[10];
    char* p = buf;
    uint8_t argv_len=0;
    
    while(*p != '\0')
    {
		while(*p != ' ' && *p != '\0')
		{
            p++;
            argv_len++;
		}
        
        argv[argc] = mm_new(argv_len+1);
        strncpy(argv[argc], p-argv_len, argv_len);      
        argv[argc][argv_len] = '\0';       
        argc++;
        argv_len = 0;
        
		while(*p == ' ' && *p != '\0')
		{
            p++;
		}         
    }
    
//    for(uint8_t i=0; i<argc;i++)
//    {
//        DEBUG_PRINTF("argv%d %s\r\n", i, argv[i]);
//    }
    
    
    if ((!strcmp(argv[0], "help")) || (!strcmp(argv[0], "?"))) {
        help_shell(argc, argv);
    } else {
        bool cmd_find= false;
        for(uint8_t i=0; i<cmd_total; i++)
        {
            if(!strcmp(argv[0], shell[i].name))
            {
                shell[i].cmd(argc, argv);
                cmd_find = true;
                break;
            }
        }
        
        if(!cmd_find)
        {
            DEBUG_PRINTF("%s: command not found\r\n\n", argv[0]);
        }
    }
    
    mm_reset();
}

void help_shell(int argc, char *argv[])
{
    for(uint8_t i=0; i<cmd_total; i++)
    {
        DEBUG_PRINTF("%s\r\n", shell[i].name);
    }
    DEBUG_PRINTF("\r\n");
}

void reboot_shell(int argc, char *argv[])
{
    NVIC_SystemReset();
}


void cli_regist(const char* name, shell_cmd cmd)
{
    shell[cmd_total].name = name;
    shell[cmd_total].cmd = cmd;
    cmd_total++;
}


