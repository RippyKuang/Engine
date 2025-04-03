#include <gtk/gtk.h>
#include <cairo.h>
#include <iostream>
#include <functional>
#include <matrix.h>

namespace Engine
{
#define LINE(a, b, c, d, e)     \
    do                          \
    {                           \
        cairo_move_to(e, a, b); \
        cairo_line_to(e, c, d); \
    } while (0)
#define CUBE_LINE(a, b, c)                                                                                                \
    do                                                                                                                    \
    {                                                                                                                     \
        if (datas[a + c * 8][0] != -1 && datas[b + c * 8][0] != -1)                                                       \
            LINE(w - datas[a + c * 8][0], h - datas[a + c * 8][1], w - datas[b + c * 8][0], h - datas[b + c * 8][1], cr); \
        cairo_stroke(cr);                                                                                                 \
    } while (0)
    class GFrame
    {
    private:
        GtkWidget *window;
        GtkWidget *canvas;
        cairo_surface_t *surface = NULL;
        void clear_surface(void);
        void processData(cairo_t *);
        std::vector<Point2i> datas;
        const int w = 1280;
        const int h = 1024;
        static gboolean draw_cb(GtkWidget *, cairo_t *, gpointer);
        static gboolean time_handler(GtkWidget *);
        static gboolean configure_event_cb(GtkWidget *,
                                           GdkEventConfigure *,
                                           gpointer);

    public:
        GFrame(int argc, char *argv[]);
        void show();
        void updateData(std::vector<Point2i> data);
    };
}