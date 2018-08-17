#include "ponnector.h"

Ponnector::Ponnector(QDialog *parent):
    QDialog(parent),
    m_ui_ptr(new Ui::Ponnector),
    m_logger_ptr(new PCPLLogger(this)),
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
        m_point_cloud_processing_backend_ptr->set_point_cloud_text(settings.value("output/set_pc_txt").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_point_cloud_text(false);
    }

    if(settings.contains("output/set_pc_bin"))
    {
        m_point_cloud_processing_backend_ptr->set_point_cloud_binary(settings.value("output/set_pc_bin").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_point_cloud_binary(false);
    }

    if(settings.contains("register/set_vis"))
    {
        m_point_cloud_processing_backend_ptr->set_visualisation(settings.value("register/set_vis").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_visualisation(false);
    }

    if(settings.contains("register/set_cloudps"))
    {
        bool *ok = new bool(false);

        int cloudps = settings.value("register/set_cloudps").toString().toInt(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_point_size(cloudps);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_point_size(2);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_point_size(2);
    }

    if(settings.contains("register/set_centps"))
    {
        bool *ok = new bool(false);

        int centps = settings.value("register/set_centps").toString().toInt(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_point_size(centps);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_point_size(8);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_point_size(8);
    }

    if(settings.contains("register/set_cloud1r"))
    {
        bool *ok = new bool(false);

        unsigned char cloud1r = static_cast<unsigned char>(settings.value("register/set_cloud1r").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_r(cloud1r);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_r(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_one_r(255);
    }

    if(settings.contains("register/set_cloud1g"))
    {
        bool *ok = new bool(false);

        unsigned char cloud1g = static_cast<unsigned char>(settings.value("register/set_cloud1g").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_g(cloud1g);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_g(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_one_g(0);
    }

    if(settings.contains("register/set_cloud1b"))
    {
        bool *ok = new bool(false);

        unsigned char cloud1b = static_cast<unsigned char>(settings.value("register/set_cloud1b").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_b(cloud1b);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_one_b(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_one_b(0);
    }

    if(settings.contains("register/set_cloud2r"))
    {
        bool *ok = new bool(false);

        unsigned char cloud2r = static_cast<unsigned char>(settings.value("register/set_cloud2r").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_r(cloud2r);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_r(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_two_r(0);
    }

    if(settings.contains("register/set_cloud2g"))
    {
        bool *ok = new bool(false);

        unsigned char cloud2g = static_cast<unsigned char>(settings.value("register/set_cloud2g").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_g(cloud2g);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_g(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_two_g(0);
    }

    if(settings.contains("register/set_cloud2b"))
    {
        bool *ok = new bool(false);

        unsigned char cloud2b = static_cast<unsigned char>(settings.value("register/set_cloud2b").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_b(cloud2b);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_cloud_two_b(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_cloud_two_b(255);
    }

    if(settings.contains("register/set_cent1r"))
    {
        bool *ok = new bool(false);

        unsigned char cent1r = static_cast<unsigned char>(settings.value("register/set_cent1r").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_r(cent1r);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_r(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_one_r(255);
    }

    if(settings.contains("register/set_cent1g"))
    {
        bool *ok = new bool(false);

        unsigned char cent1g = static_cast<unsigned char>(settings.value("register/set_cent1g").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_g(cent1g);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_g(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_one_g(255);
    }

    if(settings.contains("register/set_cent1b"))
    {
        bool *ok = new bool(false);

        unsigned char cent1b = static_cast<unsigned char>(settings.value("register/set_cent1b").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_b(cent1b);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_one_b(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_one_b(0);
    }

    if(settings.contains("register/set_cent2r"))
    {
        bool *ok = new bool(false);

        unsigned char cent2r = static_cast<unsigned char>(settings.value("register/set_cent2r").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_r(cent2r);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_r(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_two_r(0);
    }

    if(settings.contains("register/set_cent2g"))
    {
        bool *ok = new bool(false);

        unsigned char cent2g = static_cast<unsigned char>(settings.value("register/set_cent2g").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_g(cent2g);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_g(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_two_g(255);
    }

    if(settings.contains("register/set_cent2b"))
    {
        bool *ok = new bool(false);

        unsigned char cent2b = static_cast<unsigned char>(settings.value("register/set_cent2b").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_b(cent2b);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_centroid_two_b(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_centroid_two_b(255);
    }

    if(settings.contains("register/set_sr"))
    {
        bool *ok = new bool(false);

        unsigned char sr = static_cast<unsigned char>(settings.value("register/set_sr").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_r(sr);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_r(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_r(0);
    }

    if(settings.contains("register/set_sg"))
    {
        bool *ok = new bool(false);

        unsigned char sg = static_cast<unsigned char>(settings.value("register/set_sg").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_g(sg);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_g(255);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_g(255);
    }

    if(settings.contains("register/set_sb"))
    {
        bool *ok = new bool(false);

        unsigned char sb = static_cast<unsigned char>(settings.value("register/set_sb").toString().toUShort(ok));

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_b(sb);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_b(0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_b(0);
    }

    if(settings.contains("register/set_tr_txt"))
    {
        m_point_cloud_processing_backend_ptr->set_translation_text(settings.value("register/set_tr_txt").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_translation_text(false);
    }

    if(settings.contains("register/set_tr_bin"))
    {
        m_point_cloud_processing_backend_ptr->set_translation_binary(settings.value("register/set_tr_bin").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_translation_binary(false);
    }

    if(settings.contains("register/set_icp"))
    {
        m_point_cloud_processing_backend_ptr->set_icp(settings.value("register/set_icp").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_icp(false);
    }

    if(settings.contains("register/set_ndt"))
    {
        m_point_cloud_processing_backend_ptr->set_ndt(settings.value("register/set_ndt").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_ndt(false);
    }

    if(settings.contains("register/set_iterative"))
    {
        m_point_cloud_processing_backend_ptr->set_iterative(settings.value("register/set_iterative").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_iterative(true);
    }

    if(settings.contains("register/set_continuous"))
    {
        m_point_cloud_processing_backend_ptr->set_continuous(settings.value("register/set_continuous").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_continuous(false);
    }

    if(settings.contains("register/set_distance"))
    {
        m_point_cloud_processing_backend_ptr->set_distance(settings.value("register/set_distance").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_distance(true);
    }

    if(settings.contains("register/set_eigen"))
    {
        m_point_cloud_processing_backend_ptr->set_eigen(settings.value("register/set_eigen").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_eigen(false);
    }

    if(settings.contains("register/set_threshold"))
    {
        bool *ok = new bool(false);

        double threshold = settings.value("register/set_threshold").toString().toDouble(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_threshold(threshold);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_threshold(1000.0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_threshold(1000.0);
    }

    if(settings.contains("register/set_offset"))
    {
        bool *ok = new bool(false);

        int offset = settings.value("register/set_offset").toString().toInt(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_offset(offset);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_offset(10);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_offset(10);
    }

    if(settings.contains("register/set_fl"))
    {
        bool *ok = new bool(false);

        float fl = settings.value("register/set_fl").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_focal_length(fl);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_focal_length(0.0021f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_focal_length(0.0021f);
    }

    if(settings.contains("register/set_dm"))
    {
        bool *ok = new bool(false);

        double dm = settings.value("register/set_dm").toString().toDouble(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_distance_movement(dm);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_distance_movement(1000.0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_distance_movement(1000.0);
    }

    if(settings.contains("register/set_em"))
    {
        bool *ok = new bool(false);

        double em = settings.value("register/set_em").toString().toDouble(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_eigen_movement(em);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_eigen_movement(1000.0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_eigen_movement(1000.0);
    }

    if(settings.contains("register/set_ss"))
    {
        bool *ok = new bool(false);

        int ss = settings.value("register/set_ss").toString().toInt(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_smoothing_size(ss);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_smoothing_size(100);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_smoothing_size(100);
    }

    if(settings.contains("register/set_sd"))
    {
        bool *ok = new bool(false);

        double sd = settings.value("register/set_sd").toString().toDouble(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_smoothing_deviation(sd);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_smoothing_deviation(1.0);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_smoothing_deviation(1.0);
    }

    if(settings.contains("register/set_filter_x"))
    {
        bool *ok = new bool(false);

        float filter = settings.value("register/set_filter_x").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_filter_x(filter);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_filter_x(10.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_filter_x(10.0f);
    }

    if(settings.contains("register/set_filter_y"))
    {
        bool *ok = new bool(false);

        float filter = settings.value("register/set_filter_y").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_filter_y(filter);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_filter_y(10.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_filter_y(10.0f);
    }

    if(settings.contains("register/set_filter_z"))
    {
        bool *ok = new bool(false);

        float filter = settings.value("register/set_filter_z").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_filter_z(filter);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_filter_z(10.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_filter_z(10.0f);
    }

    if(settings.contains("register/set_te"))
    {
        bool *ok = new bool(false);

        double te = settings.value("register/set_te").toString().toDouble(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_transformation_epsilon(te);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_transformation_epsilon(0.01);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_transformation_epsilon(0.01);
    }

    if(settings.contains("register/set_iterations"))
    {
        bool *ok = new bool(false);

        int iterations = settings.value("register/set_iterations").toString().toInt(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_iterations(iterations);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_iterations(100);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_iterations(100);
    }

    if(settings.contains("register/set_manual"))
    {
        m_point_cloud_processing_backend_ptr->set_manual(settings.value("register/set_manual").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_manual(true);
    }

    if(settings.contains("register/set_auto"))
    {
        m_point_cloud_processing_backend_ptr->set_auto(settings.value("register/set_auto").toBool());
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_auto(false);
    }

    if(settings.contains("register/set_rg"))
    {
        bool *ok = new bool(false);

        float rg = settings.value("register/set_rg").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_rotation_guess(rg);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_rotation_guess(100);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_rotation_guess(0.6931f);
    }

    if(settings.contains("register/set_tgx"))
    {
        bool *ok = new bool(false);

        float tgx = settings.value("register/set_tgx").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_x(tgx);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_x(1.79387f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_translation_guess_x(1.79387f);
    }

    if(settings.contains("register/set_tgy"))
    {
        bool *ok = new bool(false);

        float tgy = settings.value("register/set_tgy").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_y(tgy);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_y(0.720047f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_translation_guess_y(0.720047f);
    }

    if(settings.contains("register/set_tgz"))
    {
        bool *ok = new bool(false);

        float tgz = settings.value("register/set_tgz").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_z(tgz);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_translation_guess_z(0.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_translation_guess_z(0.0f);
    }

    if(settings.contains("register/set_sm"))
    {
        bool *ok = new bool(false);

        float sm = settings.value("register/set_sm").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_magnitude(sm);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_magnitude(1.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_magnitude(1.0f);
    }

    if(settings.contains("register/set_sx"))
    {
        bool *ok = new bool(false);

        float sx = settings.value("register/set_sx").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_x(sx);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_x(0.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_x(0.0f);
    }

    if(settings.contains("register/set_sy"))
    {
        bool *ok = new bool(false);

        float sy = settings.value("register/set_sy").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_y(sy);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_y(0.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_y(0.0f);
    }

    if(settings.contains("register/set_sz"))
    {
        bool *ok = new bool(false);

        float sz = settings.value("register/set_sz").toString().toFloat(ok);

        if(*ok)
        {
            m_point_cloud_processing_backend_ptr->set_signal_z(sz);
        }
        else
        {
            m_point_cloud_processing_backend_ptr->set_signal_z(0.0f);
        }

        if(ok != nullptr)
        {
            delete ok;

            ok = nullptr;
        }
    }
    else
    {
        m_point_cloud_processing_backend_ptr->set_signal_z(0.0f);
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

    if(q_file_paths.size() > 0)
    {
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
        QStringList q_file_paths = QFileDialog::getOpenFileNames(
                    this,
                    "Select one or more files to open",
                    m_point_cloud_processing_backend_ptr->get_output_path().c_str(),
                    "Headers (*.pcd)");

        if(q_file_paths.size() > 0)
        {
            vector<string> std_file_paths = vector<string>(static_cast<unsigned long>(q_file_paths.size()), "");

            for(int i = 0; i < q_file_paths.size(); ++i)
            {
                std_file_paths[static_cast<unsigned long>(i)] = q_file_paths[i].toStdString();
            }

            if(m_point_cloud_processing_backend_ptr->load_pcd(std_file_paths))
            {
                m_header_loaded = false;

                m_pcl_loaded = true;
            }
        }
    }

    updateGUI_state();
}

void Ponnector::on__psh_register_clicked()
{
    m_logger_ptr->show();

    if(m_pcl_loaded)
    {
        m_point_cloud_processing_backend_ptr->registration();
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
