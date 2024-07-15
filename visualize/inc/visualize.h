#include <gtk/gtk.h>
#include <cairo.h>
#include <iostream>
#include <functional>
#include <matrix.h>

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