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
//! This is a Qt frontend for the PointCloudProcessingBackend class
//! This class calls the PointCloudProcessingBackend class,
//! collects its outputs and displays them to the user
//!
class Ponnector : public QDialog
{
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

    //! Gets the ui ptr
    inline Ui::Ponnector * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    //! Sets the ui ptr
    inline int set_ui_ptr(Ui::Ponnector *ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    //! Gets the logger ptr
    inline PCPLLogger * get_logger_ptr()
    {
        return m_logger_ptr;
    }

    //! Sets the logger ptr
    inline int set_logger_ptr(PCPLLogger *logger_ptr)
    {
        m_logger_ptr = logger_ptr;

        return 1;
    }

    //! Gets the update ptr
    inline QTimer * get_update_ptr()
    {
        return m_update_ptr;
    }

    //! Sets the update ptr
    inline int set_update_ptr(QTimer *update_ptr)
    {
        m_update_ptr = update_ptr;

        return 1;
    }

    //! Gets the point cloud processing backend ptr
    inline shared_ptr<PointCloudProcessingBackend> & get_point_cloud_processing_backend_ptr()
    {
        return m_point_cloud_processing_backend_ptr;
    }

    //! Sets the point cloud processing backend ptr
    inline int set_point_cloud_processing_backend_ptr(shared_ptr<PointCloudProcessingBackend> &point_cloud_processing_backend_ptr)
    {
        m_point_cloud_processing_backend_ptr = point_cloud_processing_backend_ptr;

        return 1;
    }

    //! Gets the output frequency value
    inline float get_output_frequency()
    {
        return m_output_frequency;
    }

    //! Sets the output frequency value
    inline int set_output_frequency(float output_frequency)
    {
        m_output_frequency = output_frequency;

        return 1;
    }

    //! Gets the output speed value
    inline float get_output_speed()
    {
        return m_output_speed;
    }

    //! Sets the output speed value
    inline int set_output_speed(float output_speed)
    {
        m_output_speed = output_speed;

        return 1;
    }

    //! Gets the write offset value
    inline unsigned char get_write_offset()
    {
        return m_write_offset;
    }

    //! Sets the write offset value
    inline int set_write_offset(unsigned char write_offset)
    {
        m_write_offset = write_offset;

        return 1;
    }

    //! Gets the header loaded bool
    inline bool get_header_loaded()
    {
        return m_header_loaded;
    }

    //! Sets the header loaded bool
    inline int set_header_loaded(bool header_loaded)
    {
        m_header_loaded = header_loaded;

        return 1;
    }

    //! Gets the pcl loaded bool
    inline bool get_pcl_loaded()
    {
        return m_pcl_loaded;
    }

    //! Gets the pcl loaded bool
    inline int set_pcl_loaded(bool pcl_loaded)
    {
        m_pcl_loaded = pcl_loaded;

        return 1;
    }

    //! Main
    int ponnector_main();

    //! Destruct remotely
    int ponnector_kill(bool);

private:

    //! Macro to indicate this is a QT object
    Q_OBJECT

    //! Pointer to the UI namespace
    Ui::Ponnector *m_ui_ptr;

    //! A window to display the log
    PCPLLogger *m_logger_ptr;

    //! Pointer to the update timer
    QTimer *m_update_ptr;

    //! Holds a pointer to the point cloud processing backend
    shared_ptr<PointCloudProcessingBackend> m_point_cloud_processing_backend_ptr;

    //! Holds the how often the output occurs in a second
    float m_output_frequency;

    //! Holds how many ms need to pass before a new output happens
    float m_output_speed;

    //! Holds how often a write should happen
    unsigned char m_write_offset;

    //! Holds if a header has been loaded
    bool m_header_loaded;

    //! Holds if a point cloud has been loaded
    bool m_pcl_loaded;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);

    //! Updates the values which store default values
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

    //! Event handler for Header button
    void on__psh_header_clicked();

    //! Event handler for PCL button
    void on__psh_pcl_clicked();

    //! Event handler for Register button
    void on__psh_register_clicked();

    //! Event handler for Paths button
    void on__psh_paths_clicked();

    //! Event handler for Log button
    void on__psh_show_log_clicked();

    //! Event handler for Settings button
    void on__psh_settings_clicked();

};

#endif // KINECTFRONTEND_H
