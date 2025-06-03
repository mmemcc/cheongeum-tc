#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_console.h>
#include <linenoise/linenoise.h>
#include <argtable3/argtable3.h>

#include <ce/util/user_command.h>
#include <ce/relay/control.h>
#include <ce/util/shared.h>

static struct {
  struct arg_str *relay_connection;
  struct arg_end *end;
} relay_args;

// 릴레이 관련 명령어 처리 함수들
static int cmd_relay_connection(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &relay_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, relay_args.end, argv[0]);
        return 1;
    }
    bool relay_connection[MAX_RELAYS] = {false};

    for (int i = 0; i < MAX_RELAYS; i++) {
        char *line = NULL;
        do {
            printf("Relay %d connected? (y/n): ", i + 1);
            line = linenoise("");
            if (!line) {
                continue;
            }
            
            if (strcasecmp(line, "y") == 0 || strcasecmp(line, "yes") == 0) {
                relay_connection[i] = true;
                break;
            } else if (strcasecmp(line, "n") == 0 || strcasecmp(line, "no") == 0) {
                relay_connection[i] = false;
                break;
            }
            
            printf("Invalid input. Please enter 'y' or 'n'\n");
            linenoiseFree(line);
        } while (1);
        
        linenoiseFree(line);
    }

    ce_global_update(&ce_relay_state_global[RELAY_COMP].relay_connection, &relay_connection[RELAY_COMP], sizeof(relay_connection[RELAY_COMP]), ce_relay_state_global[RELAY_COMP].relay_mutex);
    ce_global_update(&ce_relay_state_global[RELAY_FAN].relay_connection, &relay_connection[RELAY_FAN], sizeof(relay_connection[RELAY_FAN]), ce_relay_state_global[RELAY_FAN].relay_mutex);
    ce_global_update(&ce_relay_state_global[RELAY_DEF].relay_connection, &relay_connection[RELAY_DEF], sizeof(relay_connection[RELAY_DEF]), ce_relay_state_global[RELAY_DEF].relay_mutex);
    ce_global_update(&ce_relay_state_global[RELAY_AUX_LIGHT].relay_connection, &relay_connection[RELAY_AUX_LIGHT], sizeof(relay_connection[RELAY_AUX_LIGHT]), ce_relay_state_global[RELAY_AUX_LIGHT].relay_mutex);

    return 0;
}



void ce_cmd_register_relay(void)
{
    const esp_console_cmd_t relay_connection_cmd = {
        .command = "relay_connection",
        .help = "Configure relay connections",
        .hint = NULL,
        .func = &cmd_relay_connection,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&relay_connection_cmd));
}

esp_err_t ce_cmd_init(void)
{
    // 간단한 명령어 등록만 수행
    esp_console_register_help_command();
    ce_cmd_register_relay();
    
    printf("\nConsole commands registered successfully.\n");
    printf("Available commands:\n");
    printf("  help - Show available commands\n");
    printf("  relay_connection - Configure relay connections\n");
    printf("\nType 'help' for more information.\n");
    
    return ESP_OK;
}

// 별도의 콘솔 태스크 함수 추가
void console_task(void *pvParameters)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    
    repl_config.prompt = "cheongieum> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    
    if (esp_console_new_repl_uart(&hw_config, &repl_config, &repl) == ESP_OK) {
        esp_console_start_repl(repl);
    }
    
    vTaskDelete(NULL);
}

// 콘솔 태스크 시작 함수
esp_err_t ce_cmd_start_console_task(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(
        console_task,
        "console_task",
        4096,
        NULL,
        5,
        NULL,
        0
    );
    
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}