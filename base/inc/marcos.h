
#define CHECK_SELF_HIDDEN(p, pa, pb, pc)                                              \
    do                                                                                \
    {                                                                                 \
        if (LineIntersectTriangle(corners[p], corners[pa], corners[pb], corners[pc])) \
            visible[p] = false;                                                       \
    } while (0)
