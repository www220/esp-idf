#ifndef __DFS_POSIX_H__
#define __DFS_POSIX_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <rtthread.h>

#define O_DIRECTORY     0x200000
#define DFS_PATH_MAX    64

#define NO_WORKING_DIR  "system does not support working directory\n"
int dfs_mkfs(const char *fs_name, const char *device_name);
char *dfs_normalize_path(const char *directory, const char *filename);

#endif
