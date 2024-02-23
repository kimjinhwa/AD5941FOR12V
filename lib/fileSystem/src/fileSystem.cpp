#include "filesystem.h"
// for file system
#include <esp_spiffs.h>
#include <dirent.h>
#include <sys/stat.h>
// fnmatch defines
#define FNM_NOMATCH 1        // Match failed.
#define FNM_NOESCAPE 0x01    // Disable backslash escaping.
#define FNM_PATHNAME 0x02    // Slash must be matched by slash.
#define FNM_PERIOD 0x04      // Period must be matched by period.
#define FNM_LEADING_DIR 0x08 // Ignore /<tail> after Imatch.
#define FNM_CASEFOLD 0x10    // Case insensitive search.
#define FNM_PREFIX_DIRS 0x20 // Directory prefixes of pattern match too.
#define EOS '\0'
LittleFileSystem::LittleFileSystem(){
        outputStream = &Serial;
}

void LittleFileSystem::listDir(const char *path, char *match){
  DIR *dir = NULL;
  struct dirent *ent;
  char type;
  char size[9];
  char tpath[255];
  char tbuffer[80];
  struct stat sb;
  struct tm *tm_info;
  char *lpath = NULL;
  int statok;
  outputStream->printf( "\r\nList of Directory [%s]\r\n", path);
  outputStream->printf( "-----------------------------------\r\n");
  // Open directory
  dir = opendir(path);
  if (!dir)
  {
    outputStream->printf( "Error opening directory\r\n");
    return;
  }

  // Read directory entries
  uint64_t total = 0;
  int nfiles = 0;
  outputStream->printf( "T  Size      Date/Time         Name\r\n");
  outputStream->printf( "-----------------------------------\r\n");
  while ((ent = readdir(dir)) != NULL)
  {
    sprintf(tpath, path);
    if (path[strlen(path) - 1] != '/')
      strcat(tpath, "/");
    strcat(tpath, ent->d_name);
    tbuffer[0] = '\0';

    if ((match == NULL) || (fnmatch(match, tpath, (FNM_PERIOD)) == 0))
    {
      // Get file stat
      statok = stat(tpath, &sb);

      if (statok == 0)
      {
        tm_info = localtime(&sb.st_mtime);
        strftime(tbuffer, 80, "%d/%m/%Y %R", tm_info);
      }
      else
        sprintf(tbuffer, "                ");

      if (ent->d_type == DT_REG)
      {
        type = 'f';
        nfiles++;
        if (statok)
          strcpy(size, "       ?");
        else
        {
          total += sb.st_size;
          if (sb.st_size < (1024 * 1024))
            sprintf(size, "%8d", (int)sb.st_size);
          else if ((sb.st_size / 1024) < (1024 * 1024))
            sprintf(size, "%6dKB", (int)(sb.st_size / 1024));
          else
            sprintf(size, "%6dMB", (int)(sb.st_size / (1024 * 1024)));
        }
      }
      else
      {
        type = 'd';
        strcpy(size, "       -");
      }

      outputStream->printf( "%c  %s  %s  %s\r\n",
                   type,
                   size,
                   tbuffer,
                   ent->d_name);
    }
  }
  if (total)
  {
    outputStream->printf( "-----------------------------------\r\n");
    if (total < (1024 * 1024))
      outputStream->printf( "   %8d", (int)total);
    else if ((total / 1024) < (1024 * 1024))
      outputStream->printf( "   %6dKB", (int)(total / 1024));
    else
      outputStream->printf( "   %6dMB", (int)(total / (1024 * 1024)));
    outputStream->printf( " in %d file(s)\r\n", nfiles);
  }
  outputStream->printf( "-----------------------------------\r\n");

  closedir(dir);

  free(lpath);

  uint32_t tot = 0, used = 0;
  esp_spiffs_info(NULL, &tot, &used);
  outputStream->printf( "SPIFFS: free %d KB of %d KB\r\n", (tot - used) / 1024, tot / 1024);
  outputStream->printf( "-----------------------------------\r\n");
}
int LittleFileSystem::fnmatch(const char *pattern, const char *string, int flags)
{
  const char *stringstart;
  char c, test;

  for (stringstart = string;;)
    switch (c = *pattern++)
    {
    case EOS:
      if ((flags & FNM_LEADING_DIR) && *string == '/')
        return (0);
      return (*string == EOS ? 0 : FNM_NOMATCH);
    case '?':
      if (*string == EOS)
        return (FNM_NOMATCH);
      if (*string == '/' && (flags & FNM_PATHNAME))
        return (FNM_NOMATCH);
      if (*string == '.' && (flags & FNM_PERIOD) &&
          (string == stringstart ||
           ((flags & FNM_PATHNAME) && *(string - 1) == '/')))
        return (FNM_NOMATCH);
      ++string;
      break;
    case '*':
      c = *pattern;
      // Collapse multiple stars.
      while (c == '*')
        c = *++pattern;

      if (*string == '.' && (flags & FNM_PERIOD) &&
          (string == stringstart ||
           ((flags & FNM_PATHNAME) && *(string - 1) == '/')))
        return (FNM_NOMATCH);

      // Optimize for pattern with * at end or before /.
      if (c == EOS)
        if (flags & FNM_PATHNAME)
          return ((flags & FNM_LEADING_DIR) ||
                          strchr(string, '/') == NULL
                      ? 0
                      : FNM_NOMATCH);
        else
          return (0);
      else if ((c == '/') && (flags & FNM_PATHNAME))
      {
        if ((string = strchr(string, '/')) == NULL)
          return (FNM_NOMATCH);
        break;
      }

      // General case, use recursion.
      while ((test = *string) != EOS)
      {
        if (!fnmatch(pattern, string, flags & ~FNM_PERIOD))
          return (0);
        if ((test == '/') && (flags & FNM_PATHNAME))
          break;
        ++string;
      }
      return (FNM_NOMATCH);
    case '[':
      if (*string == EOS)
        return (FNM_NOMATCH);
      if ((*string == '/') && (flags & FNM_PATHNAME))
        return (FNM_NOMATCH);
      if ((pattern = rangematch(pattern, *string, flags)) == NULL)
        return (FNM_NOMATCH);
      ++string;
      break;
    case '\\':
      if (!(flags & FNM_NOESCAPE))
      {
        if ((c = *pattern++) == EOS)
        {
          c = '\\';
          --pattern;
        }
      }
      break;
      // FALLTHROUGH
    default:
      if (c == *string)
      {
      }
      else if ((flags & FNM_CASEFOLD) && (tolower((unsigned char)c) == tolower((unsigned char)*string)))
      {
      }
      else if ((flags & FNM_PREFIX_DIRS) && *string == EOS && ((c == '/' && string != stringstart) || (string == stringstart + 1 && *stringstart == '/')))
        return (0);
      else
        return (FNM_NOMATCH);
      string++;
      break;
    }
  // NOTREACHED
  return 0;
}
const char* LittleFileSystem::rangematch(const char *pattern, char test, int flags)
{
  int negate, ok;
  char c, c2;

  /*
   * A bracket expression starting with an unquoted circumflex
   * character produces unspecified results (IEEE 1003.2-1992,
   * 3.13.2).  This implementation treats it like '!', for
   * consistency with the regular expression syntax.
   * J.T. Conklin (conklin@ngai.kaleida.com)
   */
  if ((negate = (*pattern == '!' || *pattern == '^')))
    ++pattern;

  if (flags & FNM_CASEFOLD)
    test = tolower((unsigned char)test);

  for (ok = 0; (c = *pattern++) != ']';)
  {
    if (c == '\\' && !(flags & FNM_NOESCAPE))
      c = *pattern++;
    if (c == EOS)
      return (NULL);

    if (flags & FNM_CASEFOLD)
      c = tolower((unsigned char)c);

    if (*pattern == '-' && (c2 = *(pattern + 1)) != EOS && c2 != ']')
    {
      pattern += 2;
      if (c2 == '\\' && !(flags & FNM_NOESCAPE))
        c2 = *pattern++;
      if (c2 == EOS)
        return (NULL);

      if (flags & FNM_CASEFOLD)
        c2 = tolower((unsigned char)c2);

      if ((unsigned char)c <= (unsigned char)test &&
          (unsigned char)test <= (unsigned char)c2)
        ok = 1;
    }
    else if (c == test)
      ok = 1;
  }
  return (ok == negate ? NULL : pattern);
}
void LittleFileSystem::setOutputStream(Print* stream){
    this->outputStream = stream;
}
int LittleFileSystem::rm(String fileName)
{
    String unLinkfilename = String("/spiffs/") + fileName;
    if (unlink(unLinkfilename.c_str()) == -1){
      outputStream->printf("Faild to delete %s\r\n", unLinkfilename.c_str());
      return -1 ;
    }
    else{
      outputStream->printf("File deleted %s\r\n", unLinkfilename.c_str());
      return 1 ;
    }

  //
  DIR *dir = NULL;
  dir = opendir("/spiffs/");
  if (!dir)
  {
    outputStream->printf( "Error opening directory\r\n");
    return -1;
  }
  struct dirent *entry;
  unLinkfilename.replace("*", ".");
  unLinkfilename.replace("..", ".");
  if (!unLinkfilename.startsWith("."))
  {
    unLinkfilename= "." + unLinkfilename;
  }
  size_t ext_len = unLinkfilename.length();

  while ((entry = readdir(dir)) != NULL)
  {
    if (entry->d_type == DT_REG && strlen(entry->d_name) > ext_len &&
        strcmp(entry->d_name + strlen(entry->d_name) - ext_len, unLinkfilename.c_str()) == 0)
    {
      String filePath = "/spiffs/" + String(entry->d_name);
      if (unlink(filePath.c_str()) != 0)
      {
      }
      outputStream->printf( "deleted file %s\r\n", filePath.c_str());
    }
  }
}
void LittleFileSystem::littleFsInitFast(int bformat){
  esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (bformat)
  {
    esp_spiffs_format(conf.partition_label);
  }
  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      outputStream->printf("Failed to mount or format filesystem\r\n");
    }
    else if (ret == ESP_ERR_NOT_FOUND)
    {
      outputStream->printf("Failed to find SPIFFS partition\r\n");
    }
    else
    {
      outputStream->printf("Failed to initialize SPIFFS (%s)\r\n", esp_err_to_name(ret));
    }
    return;
  }
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(conf.partition_label, &total, &used);
  if (ret != ESP_OK)
  {
    outputStream->printf("\r\nFailed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
    esp_spiffs_format(conf.partition_label);
    return;
  }
  else
  {
    outputStream->printf("\r\nPartition size: total: %d, used: %d", total, used);
  }

}
void LittleFileSystem::littleFsInit(int bformat)
{
  esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (bformat)
  {
    esp_spiffs_format(conf.partition_label);
  }
  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      outputStream->printf("Failed to mount or format filesystem\r\n");
    }
    else if (ret == ESP_ERR_NOT_FOUND)
    {
      outputStream->printf("Failed to find SPIFFS partition\r\n");
    }
    else
    {
      outputStream->printf("Failed to initialize SPIFFS (%s)\r\n", esp_err_to_name(ret));
    }
    return;
  }
  outputStream->printf("\r\nPerforming SPIFFS_check().");
  ret = esp_spiffs_check(conf.partition_label);
  if (ret != ESP_OK)
  {
    outputStream->printf("\r\nSPIFFS_check() failed (%s)", esp_err_to_name(ret));
    return;
  }
  else
  {
    outputStream->printf("\r\nSPIFFS_check() successful");
  }
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(conf.partition_label, &total, &used);
  if (ret != ESP_OK)
  {
    outputStream->printf("\r\nFailed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
    esp_spiffs_format(conf.partition_label);
    return;
  }
  else
  {
    outputStream->printf("\r\nPartition size: total: %d, used: %d", total, used);
  }
}

extern TaskHandle_t *h_pxblueToothTask;
void LittleFileSystem::df()
{
  outputStream->printf( "\r\nESP32 Partition table:\r\n");
  outputStream->printf( "| Type | Sub |  Offset  |   Size   |       Label      |\r\n");
  outputStream->printf( "| ---- | --- | -------- | -------- | ---------------- |\r\n");

  esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  if (pi != NULL)
  {
    do
    {
      const esp_partition_t *p = esp_partition_get(pi);
      outputStream->printf( "|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s |\r\n",
                   p->type, p->subtype, p->address, p->size, p->label);
    } while (pi = (esp_partition_next(pi)));
  }
  outputStream->printf( "\r\n|HEAP     |       |          |   %d | ESP.getHeapSize |\r\n", ESP.getHeapSize());
  outputStream->printf( "|Free heap|       |          |   %d | ESP.getFreeHeap |\r\n", ESP.getFreeHeap());
  outputStream->printf( "|Psram    |       |          |   %d | ESP.PsramSize   |\r\n", ESP.getPsramSize());
  outputStream->printf( "|Free Psrm|       |          |   %d | ESP.FreePsram   |\r\n", ESP.getFreePsram());
  outputStream->printf( "|UsedPsram|       |          |   %d | Psram - FreeRam |\r\n", ESP.getPsramSize() - ESP.getFreePsram());
  outputStream->printf( "|BlEh Size|       |          |   %d |\r\n", uxTaskGetStackHighWaterMark(h_pxblueToothTask));
}
int LittleFileSystem::format()
{
  littleFsInit(1);
  return 1;
}


int LittleFileSystem::writeLog(time_t logtime,u_int16_t status,u_int16_t fault)
{
  FILE *fp;
  upslog_t log = {
    .logTime = logtime, .status = status, .fault = fault
  };
  fp = fopen("/spiffs/logFile.txt", "w+");
  if(fp == NULL){
    outputStream->printf("\nLogFile Open Error");
    return -1;
  };
  fwrite((upslog_t *)&log,sizeof(upslog_t),1,fp);
  fclose(fp);
  return 0;
}
void LittleFileSystem::cat(String filename)
{
  FILE *f;
  f = fopen(filename.c_str(), "r");
  if (f == NULL)
  {
    outputStream->printf("Failed to open file for reading\r\n");
    return;
  }
  char line[64];
  while (fgets(line, sizeof(line), f))
  {
    outputStream->printf("%s", line);
  }
  outputStream->printf("\r\n");

  fclose(f);
}