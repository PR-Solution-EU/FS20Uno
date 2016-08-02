
int stricmp(const char * pStr1, const char *pStr2)
{
    char c1, c2;
    int v;

    do {
        c1 = *pStr1++;
        c2 = *pStr2++;
        v = (unsigned int) tolower(c1) - (unsigned int) tolower(c2);
    } while ((v == 0) && (c1 != '\0') && (c2 != '\0') );

    return v;
}

int strnicmp(const char *pStr1, const char *pStr2, size_t Count)
{
    char c1, c2;
    int v;

    if (Count == 0)
        return 0;

    do {
        c1 = *pStr1++;
        c2 = *pStr2++;
        /* the casts are necessary when pStr1 is shorter & char is signed */
        v = (unsigned int) tolower(c1) - (unsigned int) tolower(c2);
    } while ((v == 0) && (c1 != '\0') && (--Count > 0));

    return v;
}
