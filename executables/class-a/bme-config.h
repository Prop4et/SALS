#define PIN_FORMAT_OUTPUT   16
#define PIN_FORMAT_INPUT    17
#define SAVE_INTERVAL       12 /*number of readings before saving the state, each reading happens in an interval of 5 minutes*/

const char* state_file_name = "state_file.config";
/**
 * @brief saves the file on littlefs afters some time has passed
 * 
 * @param last_saved time elapsed since last save in seconds
 * @return true if save
 * @return false otherwise 
 */
void save_state_file();