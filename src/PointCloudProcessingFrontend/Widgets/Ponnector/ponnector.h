#ifndef PONNECTOR_H
#define PONNECTOR_H

#include <QDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QSettings>
#include <QTimer>

#include <memory>

#include "src/include/PointCloudProcessingBackend.h"
#include "ui_ponnector.h"
#include "src/PointCloudProcessingFrontend/Widgets/Logger/logger.h"
#include "ponnector_settings.h"

using namespace std;

//! Used by Qt GUI
namespace Ui
{
class Ponnector;
}

//!
//! \class Ponnector
//! \brief The Ponnector class.
//!
class Ponnector : public QDialog
{
    Q_OBJECT

public:
    //! Constructor
    explicit Ponnector(QDialog *parent = nullptr);

    //! Destructor
    ~Ponnector();

    //! Copy and move constructos and assignment opperators,
    Ponnector(Ponnector &);
    Ponnector & operator = (Ponnector &);
    Ponnector(Ponnector &&);
    Ponnector & operator = (Ponnector &&);

    inline Ui::Ponnector * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    inline int set_ui_ptr(Ui::Ponnector *ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    inline Logger * get_logger_ptr()
    {
        return m_logger_ptr;
    }

    inline int set_logger_ptr(Logger *logger_ptr)
    {
        m_logger_ptr = logger_ptr;

        return 1;
    }

    inline QTimer * get_update_ptr()
    {
        return m_update_ptr;
    }

    inline int set_update_ptr(QTimer *update_ptr)
    {
        m_update_ptr = update_ptr;

        return 1;
    }

    inline shared_ptr<PointCloudProcessingBackend> & get_point_cloud_processing_backend_ptr()
    {
        return m_point_cloud_processing_backend_ptr;
    }

    inline int set_point_cloud_processing_backend_ptr(shared_ptr<PointCloudProcessingBackend> &point_cloud_processing_backend_ptr)
    {
        m_point_cloud_processing_backend_ptr = point_cloud_processing_backend_ptr;

        return 1;
    }

    inline float get_output_frequency()
    {
        return m_output_frequency;
    }

    inline int set_output_frequency(float output_frequency)
    {
        m_output_frequency = output_frequency;

        return 1;
    }

    inline float get_output_speed()
    {
        return m_output_speed;
    }

    inline int set_output_speed(float output_speed)
    {
        m_output_speed = output_speed;

        return 1;
    }

    inline unsigned char get_write_offset()
    {
        return m_write_offset;
    }

    inline int set_write_offset(unsigned char write_offset)
    {
        m_write_offset = write_offset;

        return 1;
    }

    inline bool get_loaded()
    {
        return m_loaded;
    }

    inline int set_loaded(bool loaded)
    {
        m_loaded = loaded;

        return 1;
    }

    int ponnector_main();

    int ponnector_kill(bool);

private:

    //! Pointer to the UI namespace
    Ui::Ponnector *m_ui_ptr;

    //! A window to display the log
    Logger *m_logger_ptr;

    //! Pointer to the update timer
    QTimer *m_update_ptr;

    shared_ptr<PointCloudProcessingBackend> m_point_cloud_processing_backend_ptr;

    float m_output_frequency;

    float m_output_speed;

    unsigned char m_write_offset;

    bool m_loaded;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);

    int update_settings();

    //! Updates the text in the UI
    int update_output();

signals:

private slots:

    //! Called by timer on timeout
    void update();

    //! Updates the state of the GUI
    //! Two states considered:
    //! - Loaded
    //! - Not Loaded
    void updateGUI_state();

    void on__psh_load_clicked();

    void on__psh_register_clicked();

    void on__psh_paths_clicked();

    void on__psh_show_log_clicked();

    void on__psh_settings_clicked();
};

#endif // KINECTFRONTEND_H
