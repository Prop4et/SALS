#define PIN_FORMAT_OUTPUT   16
#define PIN_FORMAT_INPUT    17
#define SAVE_INTERVAL       6*24 /*number of readings before saving the state, each reading happens in an interval of 5 minutes*/

const char* state_file_name = "state_file.config";
const char* log_file_name = "file.log";
/**
 * @brief saves the file on littlefs afters some time has passed
 * 
 */
void save_state_file();

/**
 * @brief saves the error code on the logs so that it can be debugged on a later start
 * 
 * @param string message
 * @param len message length
 */
void save_log_file(char* string, lfs_size_t len);