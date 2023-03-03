#define PIN_FORMAT_OUTPUT   16
#define PIN_FORMAT_INPUT    17
#define SAVE_INTERVAL       72  //21600/300

const char* state_file_name = "state_file.config";
const char* config_file_name = "2022_05_17_01_09_bsec_h2s_nonh2s_2_2_0_0.config"; 
/**
 * @brief saves the file on littlefs afters some time has passed
 * 
 * @param last_saved time elapsed since last save in seconds
 * @return true if save
 * @return false otherwise 
 */
void save_state_file();