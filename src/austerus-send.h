#define NORMAL					0
#define STREAM					1
#define BAR_WIDTH				35
#define PIPE_LINE_BUFFER_LEN	100
#define MESSAGE_BUFFER_LEN		250

#define INFO					0
#define	WARN					1
#define ERROR					2

void print_time(int seconds);
void print_status(int pct, int taken, int estimate);
void print_message(int mode, int errorLevel, char *fmt, ...);
ssize_t filter_comments(char *line);
int print_file(FILE *stream_input, size_t lines, const char *cmd,
	unsigned int filament, unsigned int *table, int mode, int verbose);
int main();
