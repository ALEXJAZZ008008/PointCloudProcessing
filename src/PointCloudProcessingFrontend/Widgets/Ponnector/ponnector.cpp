#include "ponnector.h"

Ponnector::Ponnector(QDialog *parent):
    QDialog(parent),
    m_ui_ptr(new Ui::Ponnector),
    m_logger_ptr(new Logger(this)),
    m_update_ptr(new QTimer(this)),
    m_point_cloud_processing_backend_ptr(new PointCloudProcessingBackend()),
    m_output_frequency(30.0f),
    m_output_speed(1000.0f / m_output_frequency),
    m_write_offset(0),
    m_header_loaded(false),
    m_pcl_loaded(false)
{

}

Ponnector::~Ponnector()
{
    destructor(true);
}

Ponnector::Ponnector(Ponnector &ponnector_ref):
    m_ui_ptr(ponnector_ref.get_ui_ptr()),
    m_logger_ptr(ponnector_ref.get_logger_ptr()),
    m_update_ptr(ponnector_ref.get_update_ptr()),
    m_point_cloud_processing_backend_ptr(ponnector_ref.get_point_cloud_processing_backend_ptr()),
    m_output_frequency(ponnector_ref.get_output_frequency()),
    m_output_speed(ponnector_ref.get_output_speed()),
    m_write_offset(ponnector_ref.get_write_offset()),
    m_header_loaded(ponnector_ref.get_header_loaded()),
    m_pcl_loaded(ponnector_ref.get_pcl_loaded())
{

}

Ponnector & Ponnector::operator = (Ponnector &ponnector_ref)
{
    m_ui_ptr = ponnector_ref.get_ui_ptr();
    m_logger_ptr = ponnector_ref.get_logger_ptr();
    m_update_ptr = ponnector_ref.get_update_ptr();
    m_point_cloud_processing_backend_ptr = ponnector_ref.get_point_cloud_processing_backend_ptr();
    m_output_frequency = ponnector_ref.get_output_frequency();
    m_output_speed = ponnector_ref.get_output_speed();
    m_write_offset = ponnector_ref.get_write_offset();
    m_header_loaded = ponnector_ref.get_header_loaded();
    m_pcl_loaded = ponnector_ref.get_pcl_loaded();

    return *this;
}

Ponnector::Ponnector(Ponnector &&ponnector_ref_ref):
    m_ui_ptr(ponnector_ref_ref.get_ui_ptr()),
    m_logger_ptr(ponnector_ref_ref.get_logger_ptr()),
    m_update_ptr(ponnector_ref_ref.get_update_ptr()),
    m_point_cloud_processing_backend_ptr(ponnector_ref_ref.get_point_cloud_processing_backend_ptr()),
    m_output_frequency(ponnector_ref_ref.get_output_frequency()),
    m_output_speed(ponnector_ref_ref.get_output_speed()),
    m_write_offset(ponnector_ref_ref.get_write_offset()),
    m_header_loaded(ponnector_ref_ref.get_header_loaded()),
    m_pcl_loaded(ponnector_ref_ref.get_pcl_loaded())
{

}

Ponnector & Ponnector::operator = (Ponnector &&ponnector_ref_ref)
{
    m_ui_ptr = ponnector_ref_ref.get_ui_ptr();
    m_logger_ptr = ponnector_ref_ref.get_logger_ptr();
    m_update_ptr = ponnector_ref_ref.get_update_ptr();
    m_point_cloud_processing_backend_ptr = ponnector_ref_ref.get_point_cloud_processing_backend_ptr();
    m_output_frequency = ponnector_ref_ref.get_output_frequency();
    m_output_speed = ponnector_ref_ref.get_output_speed();
    m_write_offset = ponnector_ref_ref.get_write_offset();
    m_header_loaded = ponnector_ref_ref.get_header_loaded();
    m_pcl_loaded = ponnector_ref_ref.get_pcl_loaded();

    return *this;
}

int Ponnector::ponnector_main()
{
    m_ui_ptr->setupUi(this);

    m_logger_ptr->setWindowFlag(Qt::Window);

    connect(m_update_ptr, SIGNAL(timeout()), this, SLOT(update()));
    m_update_ptr->start(static_cast<int>(m_output_frequency));

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
        m_update_ptr->stop();
    }

    if(m_ui_ptr != nullptr)
    {
        delete m_ui_ptr;

        m_ui_ptr = nullptr;
    }

    if(m_logger_ptr != nullptr)
    {
        delete m_logger_ptr;

        m_logger_ptr = nullptr;
    }

    if(m_update_ptr != nullptr)
    {
        delete m_update_ptr;

        m_update_ptr = nullptr;
    }

    if(m_point_cloud_processing_backend_ptr != nullptr)
    {
        m_point_cloud_processing_backend_ptr = nullptr;
    }

    return 1;
}

int Ponnector::update_settings()
{
    QSettings settings;

    if(settings.contains("defaults/output_path"))
    {
        m_point_cloud_processing_backend_ptr->set_output_path(settings.value("defaults/output_path").toString().toStdString());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_output_path(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation).toStdString());
    }

    if(settings.contains("defaults/input_path"))
    {
        m_point_cloud_processing_backend_ptr->set_input_path(settings.value("defaults/input_path").toString().toStdString());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_input_path(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation).toStdString());
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

    return 1;
}

int Ponnector::update_output()
{
    m_logger_ptr->print(m_point_cloud_processing_backend_ptr->get_log());

    m_point_cloud_processing_backend_ptr->get_log() = "";

    return 1;
}

void Ponnector::updateGUI_state()
{
    m_ui_ptr->_psh_register->setEnabled(m_pcl_loaded);
}

void Ponnector::update()
{
    if(m_write_offset >= m_output_frequency)
    {
        if(m_logger_ptr->isVisible())
        {
            update_output();
        }

        m_write_offset = 0;
    }
    else
    {
        ++m_write_offset;
    }
}

void Ponnector::on__psh_header_clicked()
{
    m_logger_ptr->show();

    QStringList q_file_paths = QFileDialog::getOpenFileNames(
                this,
                "Select one or more files to open",
                m_point_cloud_processing_backend_ptr->get_input_path().c_str(),
                "Headers (*.kpclp)");

    vector<string> std_file_paths = vector<string>(static_cast<unsigned long>(q_file_paths.size()), "");

    for(int i = 0; i < q_file_paths.size(); ++i)
    {
        std_file_paths[static_cast<unsigned long>(i)] = q_file_paths[i].toStdString();
    }

    if(m_point_cloud_processing_backend_ptr->load_headers(std_file_paths))
    {
        m_header_loaded = true;

        m_pcl_loaded = false;
    }

    updateGUI_state();
}

void Ponnector::on__psh_pcl_clicked()
{
    m_logger_ptr->show();

    if(m_header_loaded)
    {
        if(m_point_cloud_processing_backend_ptr->load_data())
        {
            if(m_point_cloud_processing_backend_ptr->calculate_point_cloud())
            {
                m_pcl_loaded = true;

                m_point_cloud_processing_backend_ptr->write_point_cloud_to_file();
            }
        }
    }
    else
    {

    }

    updateGUI_state();
}

void Ponnector::on__psh_register_clicked()
{
    if(m_pcl_loaded)
    {

    }

    updateGUI_state();
}

void Ponnector::on__psh_paths_clicked()
{
    m_point_cloud_processing_backend_ptr->get_output_path() =
            QFileDialog::getExistingDirectory(this,
                                              tr("Select the output path."),
                                              QString::fromStdString(m_point_cloud_processing_backend_ptr->get_output_path()),
                                              QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toStdString();

    m_point_cloud_processing_backend_ptr->get_input_path() =
            QFileDialog::getExistingDirectory(this,
                                              tr("Select the input path."),
                                              QString::fromStdString(m_point_cloud_processing_backend_ptr->get_input_path()),
                                              QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toStdString();

    updateGUI_state();
}

void Ponnector::on__psh_show_log_clicked()
{
    if (!m_logger_ptr->isVisible())
    {
        m_logger_ptr->show();
    }
    else
    {
        m_logger_ptr->hide();
    }

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
