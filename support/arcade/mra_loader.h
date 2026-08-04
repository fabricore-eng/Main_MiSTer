#ifndef ROMUTILS_H_
#define ROMUTILS_H_

int arcade_send_rom(const char *xml);
int xml_load(const char *xml);
void arcade_check_error();

struct dip_struct
{
	int start;
	int size;
	int num;
	int has_val;
	uint64_t mask;
	char name[32];
	char id[32][32];
	uint64_t val[32];
};

struct sw_struct
{
	char name[1024];
	int dip_num;
	uint64_t dip_def;
	uint64_t dip_cur;
	uint64_t dip_saved;
	dip_struct dip[64];
};

#define MGL_ACTION_LOAD  0
#define MGL_ACTION_RESET 1

struct mgl_item_struct
{
	char path[1024];
	int  delay;
	char type;
	union
	{
		int  index;
		int  hold;
	};
	int  valid;
	int  submenu;
	int  action;
};

// Raised from 6. Six was enough for a console .mgl (core + rom + a save slot), but a
// disc-based arcade launch legitimately needs more: BIOS, NVRAM, two security-cassette
// halves, the disc itself and a writable flash save is already six, before any reset item.
// The old cap did not report the overflow -- scan_mgl() simply stopped recording once the
// array was full, so items 7+ vanished with no log line and the failure surfaced much later
// as "the save never rebound", which is expensive to trace back to a silently dropped item.
#define kMglMaxItems 16

struct mgl_struct
{
	int  count;
	int  current;
	mgl_item_struct item[kMglMaxItems];
	uint32_t timer;
	int  state;
	int  done;
};

sw_struct *arcade_sw();
void arcade_sw_send();
void arcade_sw_save();
void arcade_sw_load();

// Read any mra info necessary for ini processing
void arcade_pre_parse(const char *xml);

bool arcade_is_vertical();
int arcade_get_direction();

void arcade_nvm_save();

// Mount the <image> elements declared by the current .mra: any image the core exposes as
// an S-slot (a CD/CHD for disc-based arcade hardware, a writable save image, an HDD...).
// Called at the end of arcade_send_rom(), after the ROM data and the DIP switches, because
// a disc board reads its straps and boots its BIOS before it ever touches the drive.
void arcade_image_mount();

mgl_struct* mgl_parse(const char *xml);
mgl_struct* mgl_get();

#endif
