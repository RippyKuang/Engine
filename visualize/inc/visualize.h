#include <gtk/gtk.h>
#include <cairo.h>
#include <iostream>
#include <functional>
#include <matrix.h>
#include <mutex>
#include <future>
#include <gdk/gdkkeysyms.h>

namespace Engine
{
#define LINE(a, b, c, d, e)     \
    do                          \
    {                           \
        cairo_move_to(e, a, b); \
        cairo_line_to(e, c, d); \
    } while (0)

    class GFrame
    {
    private:
        GtkWidget *window;
        GtkWidget *canvas;
        cairo_surface_t *surface = NULL;
        void clear_surface(void);
        void processData(cairo_t *);
        std::vector<pixel> datas;
        std::vector<Point2i> frame_datas;
        std::future<std::vector<pixel>> fut;
        std::function<void(_T)> cam_handle;
        std::mutex m;
        const int w;
        const int h;
        static gboolean draw_cb(GtkWidget *, cairo_t *, gpointer);
        static gboolean time_handler(GtkWidget *);
        static gboolean configure_event_cb(GtkWidget *,
                                           GdkEventConfigure *,
                                           gpointer);
        static gboolean deal_key_press(GtkWidget *widget, GdkEventKey *event, gpointer data);

    public:
        GFrame(int argc, char *argv[], int w, int h);
        void show(std::function<void(_T)> cam_handle);
        void updateData(std::vector<pixel> &data, std::vector<Point2i> frame_data = {});
        void updateFuture(std::future<std::vector<pixel>> &&fut, std::vector<Point2i> frame_data = {});
    };
}