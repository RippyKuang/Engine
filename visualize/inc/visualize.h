#include <gtk/gtk.h>
#include <cairo.h>
#include <iostream>
#include <functional>

namespace Engine
{
class GFrame
{
private:
    GtkWidget *window;
    GtkWidget *canvas;
    cairo_surface_t *surface = NULL;
    void clear_surface(void);
    void processData(cairo_t *);

    static gboolean draw_cb(GtkWidget *, cairo_t *, gpointer);
    static gboolean time_handler(GtkWidget *);
    static gboolean configure_event_cb(GtkWidget *,
                                       GdkEventConfigure *,
                                       gpointer);

public:
    int x = 0;
    GFrame(int argc, char *argv[]);
    void show();
};
}