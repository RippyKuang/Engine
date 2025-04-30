#include <visualize.h>
#include <thread>

namespace Engine
{
    void GFrame::clear_surface(void)
    {
        cairo_t *cr;
        cr = cairo_create(surface);
        cairo_set_source_rgb(cr, 1, 1, 1);
        cairo_paint(cr);
        cairo_destroy(cr);
    }

    gboolean GFrame::draw_cb(GtkWidget *widget, cairo_t *cr, gpointer data)
    {
        GFrame *gf = (GFrame *)(data);
        gf->clear_surface();
        cairo_t *temp_cr;
        temp_cr = cairo_create(gf->surface);
        gf->processData(temp_cr);
        cairo_set_source_surface(cr, gf->surface, 0, 0);
        cairo_paint(cr);
        return FALSE;
    }

    gboolean GFrame::time_handler(GtkWidget *widget)
    {

        if (gtk_widget_get_window(widget) == NULL)
            return FALSE;
        gtk_widget_queue_draw(widget);
        return TRUE;
    }
    gboolean GFrame::configure_event_cb(GtkWidget *widget,
                                        GdkEventConfigure *event,
                                        gpointer data)
    {
        GFrame *gf = (GFrame *)(data);
        if (gf->surface)
            cairo_surface_destroy(gf->surface);
        gf->surface = gdk_window_create_similar_surface(gtk_widget_get_window(widget),
                                                        CAIRO_CONTENT_COLOR,
                                                        gtk_widget_get_allocated_width(widget),
                                                        gtk_widget_get_allocated_height(widget));
        gf->clear_surface();
        return TRUE;
    }

    void GFrame::show()
    {
        gtk_widget_show(this->window);
        gtk_widget_show(this->canvas);
        time_handler(this->window);
        g_signal_connect(this->window, "destroy",
                         G_CALLBACK(gtk_main_quit), NULL);

        std::thread t(gtk_main);
        t.detach();
    }
    void GFrame::processData(cairo_t *cr)
    {
        std::lock_guard<std::mutex> lock(m);
        cairo_set_source_rgb(cr, 0, 0, 0);
        for (auto data : datas)
        {
            cairo_arc(cr, w - data[0], h - data[1], 1, 0, 2 * G_PI);
            cairo_fill(cr);
        }
        if (this->frame_datas.size() != 0)
        {
            cairo_set_source_rgb(cr, 255, 0, 0);
            for (auto data : frame_datas)
            {
                cairo_arc(cr, w - data[0], h - data[1], 5, 0, 2 * G_PI);
                cairo_fill(cr);
            }
            for (int i = 0; i < frame_datas.size() / 4; i++)
            {
                LINE(w - frame_datas[4 * i][0], h - frame_datas[4 * i][1], w - frame_datas[4 * i + 1][0], h - frame_datas[4 * i + 1][1], cr);
                LINE(w - frame_datas[4 * i][0], h - frame_datas[4 * i][1], w - frame_datas[4 * i + 2][0], h - frame_datas[4 * i + 2][1], cr);
                LINE(w - frame_datas[4 * i][0], h - frame_datas[4 * i][1], w - frame_datas[4 * i + 3][0], h - frame_datas[4 * i + 3][1], cr);
            }
            cairo_stroke(cr);
         //   this->frame_datas.clear();
        }

        cairo_destroy(cr);
    }
    void GFrame::updateData(std::vector<Point2i> &data, std::vector<Point2i> frame_data)
    {
        std::lock_guard<std::mutex> lock(m);
        this->datas = data;
        if (frame_data.size() == 0)
            return;
        this->frame_datas = frame_data;
    }

    GFrame::GFrame(int argc, char *argv[])
    {
        gtk_init(&argc, &argv);
        this->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(this->window), "Center");
        gtk_window_set_default_size(GTK_WINDOW(this->window), w, h);
        gtk_window_set_position(GTK_WINDOW(this->window), GTK_WIN_POS_CENTER);

        this->canvas = gtk_drawing_area_new();

        gtk_widget_set_size_request(this->canvas, w, h);
        gtk_container_add(GTK_CONTAINER(this->window), canvas);
        gtk_widget_add_events(this->canvas, GDK_BUTTON_PRESS_MASK);
        g_signal_connect(this->canvas, "draw",
                         G_CALLBACK(GFrame::draw_cb), this);
        g_signal_connect(this->canvas, "configure-event",
                         G_CALLBACK(GFrame::configure_event_cb), this);
        g_timeout_add(1, (GSourceFunc)(GFrame::time_handler), (gpointer)this->window);
    }
}