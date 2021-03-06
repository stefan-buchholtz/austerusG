#include <stdbool.h>


#define NONE        0
#define ABSOLUTE    1
#define RELATIVE    2


struct limit {
	float min;
	float max;
};

struct region {
	float x1;
	float x2;
	float y1;
	float y2;
};


int axis_position(const char *axes, char axis);
bool check_axis(const char *line, char target);
bool check_axes(const char *line, char *targets);
bool read_axis(const char *line, char target, float *value);
void read_move(const char *line, int mode, char axis, float *delta,
	float *position);
int read_axis_delta(const char *line, const char axis, int *mode, float *delta,
	float *position, float *offset);
float get_progress_table(unsigned int **table, size_t *lines, FILE *stream);
size_t get_extends(struct limit *bounds, const char *axes, bool deposition,
	bool physical, bool zmode, float zmin, struct region *ignore,
	bool verbose, FILE *stream);

