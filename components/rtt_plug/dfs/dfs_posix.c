#include "dfs_posix.h"
#include <errno.h>
#include <sys/stat.h>
#include "wear_levelling.h"
#include "esp_partition.h"

#ifdef DFS_USING_WORKDIR
char working_directory[DFS_PATH_MAX] = {"/"};
#endif

char *dfs_normalize_path(const char *directory, const char *filename)
{
    char *fullpath;
    char *dst0, *dst, *src;

    /* check parameters */
    RT_ASSERT(filename != RT_NULL);

#ifdef DFS_USING_WORKDIR
    if (directory == RT_NULL) /* shall use working directory */
    {
        directory = &working_directory[0];
    }
#else
    if ((directory == RT_NULL) && (filename[0] != '/'))
    {
        rt_kprintf(NO_WORKING_DIR);

        return RT_NULL;
    }
#endif

    if (filename[0] != '/') /* it's a absolute path, use it directly */
    {
        fullpath = rt_malloc(strlen(directory) + strlen(filename) + 2);

        if (fullpath == RT_NULL)
            return RT_NULL;

        /* join path and file name */
        rt_snprintf(fullpath, strlen(directory) + strlen(filename) + 2,
            "%s/%s", directory, filename);
    }
    else
    {
        fullpath = rt_strdup(filename); /* copy string */

        if (fullpath == RT_NULL)
            return RT_NULL;
    }

    src = fullpath;
    dst = fullpath;

    dst0 = dst;
    while (1)
    {
        char c = *src;

        if (c == '.')
        {
            if (!src[1]) src ++; /* '.' and ends */
            else if (src[1] == '/')
            {
                /* './' case */
                src += 2;

                while ((*src == '/') && (*src != '\0'))
                    src ++;
                continue;
            }
            else if (src[1] == '.')
            {
                if (!src[2])
                {
                    /* '..' and ends case */
                    src += 2;
                    goto up_one;
                }
                else if (src[2] == '/')
                {
                    /* '../' case */
                    src += 3;

                    while ((*src == '/') && (*src != '\0'))
                        src ++;
                    goto up_one;
                }
            }
        }

        /* copy up the next '/' and erase all '/' */
        while ((c = *src++) != '\0' && c != '/')
            *dst ++ = c;

        if (c == '/')
        {
            *dst ++ = '/';
            while (c == '/')
                c = *src++;

            src --;
        }
        else if (!c)
            break;

        continue;

up_one:
        dst --;
        if (dst < dst0)
        {
            rt_free(fullpath);
            return RT_NULL;
        }
        while (dst0 < dst && dst[-1] != '/')
            dst --;
    }

    *dst = '\0';

    /* remove '/' in the end of path if exist */
    dst --;
    if ((dst != fullpath) && (*dst == '/'))
        *dst = '\0';

    return fullpath;
}

char *getcwd(char *buf, size_t size)
{
#ifdef DFS_USING_WORKDIR
	rt_enter_critical();
	rt_strncpy(buf, working_directory, size);
	rt_exit_critical();
#else
	rt_kprintf(NO_WORKING_DIR);
#endif
	return buf;
}

int chdir(const char *path)
{
    if (path == RT_NULL)
    {
        rt_kprintf("%s\n", working_directory);
        return 0;
    }

    if (rt_strlen(path) > DFS_PATH_MAX)
    {
        errno = ENOTDIR;
        return -1;
    }

    char *fullpath = dfs_normalize_path(NULL, path);
    if (fullpath == RT_NULL)
    {
        errno = ENOTDIR;
        return -1; /* build path failed */
    }
	
	// 默认根目录只存在spi目录
	if (fullpath[0] == '/' && fullpath[1] == '\0')
	{
		/* copy full path to working directory */
		strncpy(working_directory, fullpath, DFS_PATH_MAX);
		/* release normalize directory path name */
		rt_free(fullpath);
		
		return 0;
	}
	// 其他目录按照实际情况处理
    DIR *d = opendir(fullpath);
    if (d == RT_NULL)
    {
        rt_free(fullpath);
        return -1;
    }

    /* close directory stream */
    closedir(d);

    /* copy full path to working directory */
    strncpy(working_directory, fullpath, DFS_PATH_MAX);
    /* release normalize directory path name */
    rt_free(fullpath);

    return 0;
}

void cat(const char* filename)
{
	char *full = dfs_normalize_path(NULL, filename);
	if (full == NULL)
		return;
	int length, fd = open(full, O_RDONLY, 0);
	rt_free(full);

	if (fd < 0) {
		rt_kprintf("Open %s failed\n", filename);
		return;
	}

	do
	{
		char buffer[81] = {0};
        length = read(fd, buffer, sizeof(buffer)-1 );
        if (length > 0)
            rt_kprintf("%s", buffer);
    }while (length > 0);

    close(fd);
}

#define BUF_SZ  4096
void copy(const char *src, const char *dst)
{
    rt_uint8_t *block_ptr;
    block_ptr = rt_malloc(BUF_SZ);
    if (block_ptr == RT_NULL)
    {
        rt_kprintf("out of memory\n");
        return;
    }

	char *full = dfs_normalize_path(NULL, src);
	if (full == NULL)
		return;
	int read_bytes, src_fd = open(full, O_RDONLY, 0);
	free(full);
    if (src_fd < 0)
    {
        rt_free(block_ptr);
        rt_kprintf("Read %s failed\n", src);
        return;
    }
	full = dfs_normalize_path(NULL, dst);
	if (full == NULL){
		close(src_fd);
		return;
	}
	int dst_fd = open(full, O_WRONLY|O_CREAT, 0);
	free(full);
    if (dst_fd < 0)
    {
        close(src_fd);
        rt_free(block_ptr);
        rt_kprintf("Write %s failed\n", dst);
        return;
    }

    do
    {
        read_bytes = read(src_fd, block_ptr, BUF_SZ);
        if (read_bytes > 0)
        {
            int length = write(dst_fd, block_ptr, read_bytes);
            if (length != read_bytes)
            {
                rt_kprintf("Write file data failed, length=%d\n", length);
                break;
            }
        }
    } while (read_bytes > 0);

    close(src_fd);
    close(dst_fd);
    rt_free(block_ptr);
}

void ls(const char *pathname)
{
	// 默认根目录只存在spi目录
	if ((pathname[0] == '/' && pathname[1] == '\0') || pathname == NULL)
	{
		rt_kprintf("%-20s", "/spi");
		rt_kprintf("%-25s\n", "<DIR>");
		return;
	}
	// 其他目录按照实际情况处理
	struct dirent* ent = NULL;
 	DIR *dir = opendir(pathname);
	if (dir != NULL)
	{
		rt_kprintf("Directory %s:\n", pathname);
		while (NULL != (ent=readdir(dir)))
		{
			/* build full path for each file */
			char *fullpath = dfs_normalize_path(pathname, ent->d_name);
			if (fullpath == RT_NULL)
				break;

			/* get file type or size info */
			struct stat st;
			if (stat(fullpath, &st) == 0)
			{
				rt_kprintf("%-20s", ent->d_name);
				if (S_ISDIR(st.st_mode))
					rt_kprintf("%-25s\n", "<DIR>");
				else
					rt_kprintf("%-25lu\n", st.st_size);
			}
			else
			{
				rt_kprintf("BAD file: %s\n", ent->d_name);
			}
			rt_free(fullpath);
		}
		closedir(dir);
	}	
    else
    {
        rt_kprintf("No such directory\n");
    }
}

int dfs_mkfs(const char *fs_name, const char *device_name)
{
	int sub = (strcasecmp(fs_name,"fat")==0)?ESP_PARTITION_SUBTYPE_DATA_FAT:ESP_PARTITION_SUBTYPE_DATA_SPIFFS;
    const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, sub, device_name);
	if (part == NULL)
	{
		rt_kprintf("No such type %s part %s\n", fs_name, device_name);
		rt_kprintf("mkfs -t fat storage or mkfs -t spiffs syscfg\n", device_name);
		return RT_ERROR;
	}
	rt_kprintf("part %s begin:%x,size:%x\n", device_name, part->address, part->size);
	rt_tick_t t1 = rt_tick_get();
	esp_partition_erase_range(part, 0, part->size);
	rt_kprintf("part %s finish to erase elapsed:%d\n", device_name, rt_tick_get()-t1);
    return RT_EOK;
}
