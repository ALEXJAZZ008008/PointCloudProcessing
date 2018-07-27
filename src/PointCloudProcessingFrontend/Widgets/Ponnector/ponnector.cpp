#include "ponnector.h"

Ponnector::Ponnector(QDialog *parent):
    QDialog(parent),
    m_ui_ptr(new Ui::Ponnector),
    m_update_ptr(new QTimer(this)),
    m_acquisition_start_time(),
    m_acquisition_frequency(30.0f),
    m_acquisition_speed(1000.0f / m_acquisition_frequency),
    m_write_offset(0),
    m_is_connected(false),
    m_is_acquiring(false)
{

}

Ponnector::~Ponnector()
{
    destructor(true);
}

Ponnector::Ponnector(Ponnector &ponnector_ref):
    m_ui_ptr(ponnector_ref.get_ui_ptr()),
    m_update_ptr(ponnector_ref.get_update_ptr()),
    m_acquisition_start_time(ponnector_ref.get_acquisition_start_time()),
    m_acquisition_frequency(ponnector_ref.get_acquisition_frequency()),
    m_acquisition_speed(ponnector_ref.get_acquisition_speed()),
    m_write_offset(ponnector_ref.get_write_offset()),
    m_is_connected(ponnector_ref.get_is_connected()),
    m_is_acquiring(ponnector_ref.get_is_acquiring())
{

}

Ponnector & Ponnector::operator = (Ponnector &ponnector_ref)
{
    m_ui_ptr = ponnector_ref.get_ui_ptr();
    m_update_ptr = ponnector_ref.get_update_ptr();
    m_acquisition_start_time = ponnector_ref.get_acquisition_start_time();
    m_acquisition_frequency = ponnector_ref.get_acquisition_frequency();
    m_acquisition_speed = ponnector_ref.get_acquisition_speed();
    m_write_offset = ponnector_ref.get_write_offset();
    m_is_connected = ponnector_ref.get_is_connected();
    m_is_acquiring = ponnector_ref.get_is_acquiring();

    return *this;
}

Ponnector::Ponnector(Ponnector &&ponnector_ref_ref):
    m_ui_ptr(ponnector_ref_ref.get_ui_ptr()),
    m_update_ptr(ponnector_ref_ref.get_update_ptr()),
    m_acquisition_start_time(ponnector_ref_ref.get_acquisition_start_time()),
    m_acquisition_frequency(ponnector_ref_ref.get_acquisition_frequency()),
    m_acquisition_speed(ponnector_ref_ref.get_acquisition_speed()),
    m_write_offset(ponnector_ref_ref.get_write_offset()),
    m_is_connected(ponnector_ref_ref.get_is_connected()),
    m_is_acquiring(ponnector_ref_ref.get_is_acquiring())
{

}

Ponnector & Ponnector::operator = (Ponnector &&ponnector_ref_ref)
{
    m_ui_ptr = ponnector_ref_ref.get_ui_ptr();
    m_update_ptr = ponnector_ref_ref.get_update_ptr();
    m_acquisition_start_time = ponnector_ref_ref.get_acquisition_start_time();
    m_acquisition_frequency = ponnector_ref_ref.get_acquisition_frequency();
    m_acquisition_speed = ponnector_ref_ref.get_acquisition_speed();
    m_write_offset = ponnector_ref_ref.get_write_offset();
    m_is_connected = ponnector_ref_ref.get_is_connected();
    m_is_acquiring = ponnector_ref_ref.get_is_acquiring();

    return *this;
}

int Ponnector::ponnector_main()
{
    m_ui_ptr->setupUi(this);

    connect(this, &Ponnector::connection_status_changed, this, &Ponnector::updateGUI_state);
    connect(this, &Ponnector::camera_angle_changed, this, &Ponnector::updateGUI_state);
    connect(this, &Ponnector::acquisition_status_changed, this, &Ponnector::updateGUI_state);

    update_settings();

    updateGUI_state();

    show();

    return 1;
}

int Ponnector::ponnector_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int Ponnector::destructor(bool hard)
{
    if(hard)
    {

    }

    if(m_ui_ptr != nullptr)
    {
        delete m_ui_ptr;

        m_ui_ptr = nullptr;
    }

    if(m_update_ptr != nullptr)
    {
        delete m_update_ptr;

        m_update_ptr = nullptr;
    }

    return 1;
}

int Ponnector::update_settings()
{
    QSettings settings;

    if(settings.contains("output/set_depth_image"))
    {

    }
    else
    {

    }

    if(settings.contains("output/set_rgb_image"))
    {

    }
    else
    {

    }

    if(settings.contains("output/set_pc_txt"))
    {

    }
    else
    {

    }

    if(settings.contains("output/set_pc_bin"))
    {

    }
    else
    {

    }

    if(settings.contains("input/resolution_small"))
    {

    }
    else
    {

    }

    if(settings.contains("input/resolution_med"))
    {

    }
    else
    {

    }

    if(settings.contains("input/resolution_high"))
    {

    }
    else
    {

    }

    if(settings.contains("defaults/output_path"))
    {

    }
    else
    {

    }

    return 1;
}

int Ponnector::update_output()
{
    return 1;
}

void Ponnector::updateGUI_state()
{
    m_ui_ptr->_psh_connect->setEnabled(!m_is_connected);
    m_ui_ptr->_psh_disconnect->setEnabled(m_is_connected && !m_is_acquiring);

    m_ui_ptr->psh_tilt_up->setEnabled(m_is_connected);
    m_ui_ptr->psh_tilt_down->setEnabled(m_is_connected);
    m_ui_ptr->le_cur_tilt->setEnabled(m_is_connected);

    m_ui_ptr->_psh_acquire_start->setEnabled(m_is_connected && !m_is_acquiring);
    m_ui_ptr->_psh_acquire_stop->setEnabled(m_is_connected && m_is_acquiring);

    m_ui_ptr->_psh_output_path->setEnabled(!m_is_acquiring);
    m_ui_ptr->_psh_settings->setEnabled(!m_is_acquiring);
}

void Ponnector::update()
{
    if(m_is_connected)
    {
        if(m_write_offset >= m_acquisition_frequency)
        {
            m_ui_ptr->lbl_time_lapsed->setText(to_string(duration_cast<duration<int>>(high_resolution_clock::now() - m_acquisition_start_time).count()).c_str());

            m_write_offset = 0;
        }
        else
        {
            ++m_write_offset;
        }
    }
}

void Ponnector::on__psh_connect_clicked()
{
    if(!m_is_connected)
    {

    }

    update_output();
}

void Ponnector::on__psh_disconnect_clicked()
{
    if(m_is_connected)
    {
        if(m_is_acquiring)
        {
            m_update_ptr->stop();

            m_is_acquiring = false;
        }

        m_is_connected = false;

        emit connection_status_changed();
    }

    update_output();
}

void Ponnector::on_psh_tilt_up_clicked()
{
    if(m_is_connected)
    {
        emit camera_angle_changed();
    }

    update_output();
}

void Ponnector::on_psh_tilt_down_clicked()
{
    if(m_is_connected)
    {
        emit camera_angle_changed();
    }

    update_output();
}

void Ponnector::on_le_cur_tilt_returnPressed()
{
    if(m_is_connected)
    {
        emit camera_angle_changed();
    }

    update_output();
}

void Ponnector::on__psh_acquire_start_clicked()
{
    if(m_is_connected)
    {
        connect(m_update_ptr, SIGNAL(timeout()), this, SLOT(update()));
        m_update_ptr->start(static_cast<int>(m_acquisition_frequency));

        m_acquisition_start_time = high_resolution_clock::now();

        m_is_acquiring = true;

        emit acquisition_status_changed();
    }

    update_output();
}

void Ponnector::on__psh_acquire_stop_clicked()
{
    if(m_is_connected)
    {
        m_update_ptr->stop();

        m_is_acquiring = false;

        emit acquisition_status_changed();
    }

    update_output();
}

void Ponnector::on__psh_show_log_clicked()
{
    updateGUI_state();
}

void Ponnector::on__psh_output_path_clicked()
{
    updateGUI_state();
}

void Ponnector::on__psh_settings_clicked()
{
    Ponnector_Settings *ponnector_settings_ptr = new Ponnector_Settings(this);

    ponnector_settings_ptr->exec();

    update_settings();

    if(ponnector_settings_ptr != nullptr)
    {
        delete ponnector_settings_ptr;

        ponnector_settings_ptr = nullptr;
    }

    updateGUI_state();
}
