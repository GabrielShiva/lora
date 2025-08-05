#include "sd_card_func.h"

// Retorna um ponteiro para a estrutura sd_card_t com o nome de pcName igual à *name
// sd_get_num() -> retorna o número total de cartões sd disponíveis no sistema
// sd_get_by_num(i) -> retorna a instância do cartão sd pelo index
sd_card_t *sd_get_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i) {
        if (0 == strcmp(sd_get_by_num(i)->pcName, name)) {
            return sd_get_by_num(i);
        }
    }

    DBG_PRINTF("%s: Nome desconhecido %s\n", __func__, name);
    return NULL;
}

// Retorna um ponteiro para a estrutura do sistema de arquivos FAT associado ao cartão SD (*name)
FATFS *sd_get_fs_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i) {
        if (0 == strcmp(sd_get_by_num(i)->pcName, name)) {
            return &sd_get_by_num(i)->fatfs;
        }
    }

    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

// Realiza a formatação de um dispositivo de armazenamento identificado por um nome
bool run_format() {
    // Tenta obter o nome do dispositivo de uma entrada prévia
    // Se nenhum nome dor fornecido, define o primeiro dispositivo encontrado
    const char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        arg1 = sd_get_by_num(0)->pcName;
    }

    // busca pelo sistema de arquivos do dispositivo selecionado
    FATFS *p_fs = sd_get_fs_by_name(arg1);

    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return false;
    }

    // Realiza a formatação do dispositivo
    // 0,0 -> utiliza a formatação padrão
    // FF_MAX_SS * 2 -> define o tamanho do buffer (1024 bytes)
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);

    // Se a formatação falhar, exibe uma mensagem de erro
    if (FR_OK != fr) {
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    return true;
}

// Realiza a montaegm do sistema de arquivos do cartão SD utilizando FatFS
bool run_mount() {
    const char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        arg1 = sd_get_by_num(0)->pcName;
    }

    FATFS *p_fs = sd_get_fs_by_name(arg1);

    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return false;
    }

    FRESULT fr = f_mount(p_fs, arg1, 1);

    if (FR_OK != fr) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;

    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
    return true;
}

// Realiza a desmontagem do sistema de arquivos do cartão SD
bool run_unmount() {
    const char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        arg1 = sd_get_by_num(0)->pcName;
    }

    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return false;
    }

    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr) {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed

    printf("SD ( %s ) desmontado\n", pSD->pcName);
    return true;
}

// Retorna a quantidade total de espaço do cartão SD e a quantidade disponível
void run_get_size() {
    const char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        arg1 = sd_get_by_num(0)->pcName;
    }

    DWORD fre_clust, fre_sect, tot_sect;

    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }

    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr) {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;

    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}

// Exibe os diretórios e arquivos dentro do cartão SD
void run_ls() {
    const char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        arg1 = "";
    }

    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;

    if (arg1[0]) {
        p_dir = arg1;
    } else {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr) {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }
        p_dir = cwdbuf;
    }

    printf("Directory Listing: %s\n", p_dir);

    DIR dj;
    FILINFO fno;

    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);

    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr) {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    while (fr == FR_OK && fno.fname[0]) {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR) {
            pcAttrib = pcDirectory;
        } else if (fno.fattrib & AM_RDO) {
            pcAttrib = pcReadOnlyFile;
        } else {
            pcAttrib = pcWritableFile;
        }

        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }

    f_closedir(&dj);
}

// Exibe o conteúdo do arquivo
void run_cat() {
    char *arg1 = strtok(NULL, " ");
    if (!arg1) {
        printf("Missing argument\n");
        return;
    }

    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);

    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    char buf[256];
    while (f_gets(buf, sizeof buf, &fil)) {
        printf("%s", buf);
    }

    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename) {
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);

    if (res != FR_OK) {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");
        return;
    }

    char buffer[128];
    UINT br;

    printf("Conteúdo do arquivo %s:\n", filename);

    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0) {
        buffer[br] = '\0';
        printf("%s", buffer);
    }

    f_close(&file);

    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

