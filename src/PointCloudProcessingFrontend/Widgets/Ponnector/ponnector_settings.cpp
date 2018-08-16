#include "ponnector_settings.h"

Ponnector_Settings::Ponnector_Settings(QWidget *parent):
    QDialog(parent),
    m_ui_ptr(new Ui::Ponnector_Settings)
{
    m_ui_ptr->setupUi(this);

    QSettings settings;

    restoreGeometry(settings.value("mainWindowGeometry").toByteArray());

    if(settings.contains("defaults/output_path"))
    {
        m_ui_ptr->_le_default_output->setText(settings.value("defaults/output_path").toString());
    }
    else
    {
        m_ui_ptr->_le_default_output->setText(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
    }

    if(settings.contains("defaults/input_path"))
    {
        m_ui_ptr->_le_default_input->setText(settings.value("defaults/input_path").toString());
    }
    else
    {
        m_ui_ptr->_le_default_input->setText(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
    }

    if(settings.contains("output/set_pc_txt"))
    {
        m_ui_ptr->_chk_output_pc_txt->setChecked(settings.value("output/set_pc_txt").toBool());
    }
    else
    {
        m_ui_ptr->_chk_output_pc_txt->setChecked(false);
    }

    if(settings.contains("output/set_pc_bin"))
    {
        m_ui_ptr->_chk_output_pc_bin->setChecked(settings.value("output/set_pc_bin").toBool());
    }
    else
    {
        m_ui_ptr->_chk_output_pc_bin->setChecked(false);
    }

    if(settings.contains("register/set_vis"))
    {
        m_ui_ptr->_chk_register_vis->setChecked(settings.value("register/set_vis").toBool());
    }
    else
    {
        m_ui_ptr->_chk_register_vis->setChecked(false);
    }

    if(settings.contains("register/set_cloudps"))
    {
        m_ui_ptr->le_register_cloudps->setText(settings.value("register/set_cloudps").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloudps->setText("2");
    }

    if(settings.contains("register/set_centps"))
    {
        m_ui_ptr->le_register_centps->setText(settings.value("register/set_centps").toString());
    }
    else
    {
        m_ui_ptr->le_register_centps->setText("8");
    }

    if(settings.contains("register/set_cloud1r"))
    {
        m_ui_ptr->le_register_cloud1r->setText(settings.value("register/set_cloud1r").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud1r->setText("255");
    }

    if(settings.contains("register/set_cloud1g"))
    {
        m_ui_ptr->le_register_cloud1g->setText(settings.value("register/set_cloud1g").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud1g->setText("0");
    }

    if(settings.contains("register/set_cloud1b"))
    {
        m_ui_ptr->le_register_cloud1b->setText(settings.value("register/set_cloud1b").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud1b->setText("0");
    }

    if(settings.contains("register/set_cloud2r"))
    {
        m_ui_ptr->le_register_cloud2r->setText(settings.value("register/set_cloud2r").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud2r->setText("0");
    }

    if(settings.contains("register/set_cloud2g"))
    {
        m_ui_ptr->le_register_cloud2g->setText(settings.value("register/set_cloud2g").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud2g->setText("0");
    }

    if(settings.contains("register/set_cloud2b"))
    {
        m_ui_ptr->le_register_cloud2b->setText(settings.value("register/set_cloud2b").toString());
    }
    else
    {
        m_ui_ptr->le_register_cloud2b->setText("255");
    }

    if(settings.contains("register/set_cent1r"))
    {
        m_ui_ptr->le_register_cent1r->setText(settings.value("register/set_cent1r").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent1r->setText("255");
    }

    if(settings.contains("register/set_cent1g"))
    {
        m_ui_ptr->le_register_cent1g->setText(settings.value("register/set_cent1g").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent1g->setText("255");
    }

    if(settings.contains("register/set_cent1b"))
    {
        m_ui_ptr->le_register_cent1b->setText(settings.value("register/set_cent1b").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent1b->setText("0");
    }

    if(settings.contains("register/set_cent2r"))
    {
        m_ui_ptr->le_register_cent2r->setText(settings.value("register/set_cent2r").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent2r->setText("0");
    }

    if(settings.contains("register/set_cent2g"))
    {
        m_ui_ptr->le_register_cent2g->setText(settings.value("register/set_cent2g").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent2g->setText("255");
    }

    if(settings.contains("register/set_cent2b"))
    {
        m_ui_ptr->le_register_cent2b->setText(settings.value("register/set_cent2b").toString());
    }
    else
    {
        m_ui_ptr->le_register_cent2b->setText("255");
    }

    if(settings.contains("register/set_tr_txt"))
    {
        m_ui_ptr->_chk_register_tr_txt->setChecked(settings.value("register/set_tr_txt").toBool());
    }
    else
    {
        m_ui_ptr->_chk_register_tr_txt->setChecked(false);
    }

    if(settings.contains("register/set_tr_bin"))
    {
        m_ui_ptr->_chk_register_tr_bin->setChecked(settings.value("register/set_tr_bin").toBool());
    }
    else
    {
        m_ui_ptr->_chk_register_tr_bin->setChecked(false);
    }

    if(settings.contains("register/set_icp"))
    {
        m_ui_ptr->rb_register_icp->setChecked(settings.value("register/set_icp").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_icp->setChecked(true);
    }

    if(settings.contains("register/set_ndt"))
    {
        m_ui_ptr->rb_register_ndt->setChecked(settings.value("register/set_ndt").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_ndt->setChecked(false);
    }

    if(settings.contains("register/set_iterative"))
    {
        m_ui_ptr->rb_register_iterative->setChecked(settings.value("register/set_iterative").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_iterative->setChecked(true);
    }

    if(settings.contains("register/set_continuous"))
    {
        m_ui_ptr->rb_register_continuous->setChecked(settings.value("register/set_continuous").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_continuous->setChecked(false);
    }

    if(settings.contains("register/set_distance"))
    {
        m_ui_ptr->rb_register_distance->setChecked(settings.value("register/set_distance").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_distance->setChecked(true);
    }

    if(settings.contains("register/set_eigen"))
    {
        m_ui_ptr->rb_register_eigen->setChecked(settings.value("register/set_eigen").toBool());
    }
    else
    {
        m_ui_ptr->rb_register_eigen->setChecked(false);
    }

    if(settings.contains("register/set_threshold"))
    {
        m_ui_ptr->le_register_threshold->setText(settings.value("register/set_threshold").toString());
    }
    else
    {
        m_ui_ptr->le_register_threshold->setText("1000.0");
    }

    if(settings.contains("register/set_offset"))
    {
        m_ui_ptr->le_register_offset->setText(settings.value("register/set_offset").toString());
    }
    else
    {
        m_ui_ptr->le_register_offset->setText("10");
    }

    if(settings.contains("register/set_fl"))
    {
        m_ui_ptr->le_register_fl->setText(settings.value("register/set_fl").toString());
    }
    else
    {
        m_ui_ptr->le_register_fl->setText("0.0021");
    }

    if(settings.contains("register/set_dm"))
    {
        m_ui_ptr->le_register_dm->setText(settings.value("register/set_dm").toString());
    }
    else
    {
        m_ui_ptr->le_register_dm->setText("1000.0");
    }

    if(settings.contains("register/set_em"))
    {
        m_ui_ptr->le_register_em->setText(settings.value("register/set_em").toString());
    }
    else
    {
        m_ui_ptr->le_register_em->setText("1000.0");
    }

    if(settings.contains("register/set_ss"))
    {
        m_ui_ptr->le_register_ss->setText(settings.value("register/set_ss").toString());
    }
    else
    {
        m_ui_ptr->le_register_ss->setText("100");
    }

    if(settings.contains("register/set_sd"))
    {
        m_ui_ptr->le_register_sd->setText(settings.value("register/set_sd").toString());
    }
    else
    {
        m_ui_ptr->le_register_sd->setText("1.0");
    }

    if(settings.contains("register/set_te"))
    {
        m_ui_ptr->le_register_te->setText(settings.value("register/set_te").toString());
    }
    else
    {
        m_ui_ptr->le_register_te->setText("0.01");
    }

    if(settings.contains("register/set_iterations"))
    {
        m_ui_ptr->le_register_iterations->setText(settings.value("register/set_iterations").toString());
    }
    else
    {
        m_ui_ptr->le_register_iterations->setText("100");
    }

    if(settings.contains("register/set_rg"))
    {
        m_ui_ptr->le_register_rg->setText(settings.value("register/set_rg").toString());
    }
    else
    {
        m_ui_ptr->le_register_rg->setText("0.6931");
    }

    if(settings.contains("register/set_tgx"))
    {
        m_ui_ptr->le_register_tgx->setText(settings.value("register/set_tgx").toString());
    }
    else
    {
        m_ui_ptr->le_register_tgx->setText("1.79387");
    }

    if(settings.contains("register/set_tgy"))
    {
        m_ui_ptr->le_register_tgy->setText(settings.value("register/set_tgy").toString());
    }
    else
    {
        m_ui_ptr->le_register_tgy->setText("0.720047");
    }

    if(settings.contains("register/set_tgz"))
    {
        m_ui_ptr->le_register_tgz->setText(settings.value("register/set_tgz").toString());
    }
    else
    {
        m_ui_ptr->le_register_tgz->setText("0.0");
    }

    if(settings.contains("register/set_filter_x"))
    {
        m_ui_ptr->le_register_filter_x->setText(settings.value("register/set_filter_x").toString());
    }
    else
    {
        m_ui_ptr->le_register_filter_x->setText("10.0");
    }

    if(settings.contains("register/set_filter_y"))
    {
        m_ui_ptr->le_register_filter_y->setText(settings.value("register/set_filter_y").toString());
    }
    else
    {
        m_ui_ptr->le_register_filter_y->setText("10.0");
    }

    if(settings.contains("register/set_filter_z"))
    {
        m_ui_ptr->le_register_filter_z->setText(settings.value("register/set_filter_z").toString());
    }
    else
    {
        m_ui_ptr->le_register_filter_z->setText("10.0");
    }
}

Ponnector_Settings::~Ponnector_Settings()
{
    destructor(true);
}

Ponnector_Settings::Ponnector_Settings(Ponnector_Settings &ponector_settings_ref):
    m_ui_ptr(ponector_settings_ref.get_ui_ptr())
{

}

Ponnector_Settings & Ponnector_Settings::operator = (Ponnector_Settings &ponector_settings_ref)
{
    m_ui_ptr = ponector_settings_ref.get_ui_ptr();

    return *this;
}

Ponnector_Settings::Ponnector_Settings(Ponnector_Settings &&ponector_settings_ref_ref):
    m_ui_ptr(ponector_settings_ref_ref.get_ui_ptr())
{

}

Ponnector_Settings & Ponnector_Settings::operator = (Ponnector_Settings &&ponector_settings_ref_ref)
{
    m_ui_ptr = ponector_settings_ref_ref.get_ui_ptr();

    return *this;
}

int Ponnector_Settings::ponnector_settings_main()
{
    return 1;
}

int Ponnector_Settings::ponnector_settings_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int Ponnector_Settings::destructor(bool hard)
{
    if(hard)
    {

    }

    if(m_ui_ptr != nullptr)
    {
        delete m_ui_ptr;

        m_ui_ptr = nullptr;
    }

    return 1;
}

void Ponnector_Settings::on_buttonBox_accepted()
{
    QSettings settings;

    settings.setValue("defaults/output_path", m_ui_ptr->_le_default_output->text());
    settings.setValue("defaults/input_path", m_ui_ptr->_le_default_input->text());

    settings.setValue("output/set_pc_txt", m_ui_ptr->_chk_output_pc_txt->isChecked());
    settings.setValue("output/set_pc_bin", m_ui_ptr->_chk_output_pc_bin->isChecked());

    settings.setValue("register/set_vis", m_ui_ptr->_chk_register_vis->isChecked());
    settings.setValue("register/set_cloudps", m_ui_ptr->le_register_cloudps->text());
    settings.setValue("register/set_centps", m_ui_ptr->le_register_centps->text());
    settings.setValue("register/set_cloud1r", m_ui_ptr->le_register_cloud1r->text());
    settings.setValue("register/set_cloud1g", m_ui_ptr->le_register_cloud1g->text());
    settings.setValue("register/set_cloud1b", m_ui_ptr->le_register_cloud1b->text());
    settings.setValue("register/set_cloud2r", m_ui_ptr->le_register_cloud2r->text());
    settings.setValue("register/set_cloud2g", m_ui_ptr->le_register_cloud2g->text());
    settings.setValue("register/set_cloud2b", m_ui_ptr->le_register_cloud2b->text());
    settings.setValue("register/set_cent1r", m_ui_ptr->le_register_cent1r->text());
    settings.setValue("register/set_cent1g", m_ui_ptr->le_register_cent1g->text());
    settings.setValue("register/set_cent1b", m_ui_ptr->le_register_cent1b->text());
    settings.setValue("register/set_cent2r", m_ui_ptr->le_register_cent2r->text());
    settings.setValue("register/set_cent2g", m_ui_ptr->le_register_cent2g->text());
    settings.setValue("register/set_cent2b", m_ui_ptr->le_register_cent2b->text());

    settings.setValue("register/set_tr_txt", m_ui_ptr->_chk_register_tr_txt->isChecked());
    settings.setValue("register/set_tr_bin", m_ui_ptr->_chk_register_tr_bin->isChecked());

    settings.setValue("register/set_icp", m_ui_ptr->rb_register_icp->isChecked());
    settings.setValue("register/set_ndt", m_ui_ptr->rb_register_ndt->isChecked());
    settings.setValue("register/set_iterative", m_ui_ptr->rb_register_iterative->isChecked());
    settings.setValue("register/set_continuous", m_ui_ptr->rb_register_continuous->isChecked());
    settings.setValue("register/set_distance", m_ui_ptr->rb_register_distance->isChecked());
    settings.setValue("register/set_eigen", m_ui_ptr->rb_register_eigen->isChecked());

    settings.setValue("register/set_threshold", m_ui_ptr->le_register_threshold->text());
    settings.setValue("register/set_offset", m_ui_ptr->le_register_offset->text());
    settings.setValue("register/set_fl", m_ui_ptr->le_register_fl->text());
    settings.setValue("register/set_dm", m_ui_ptr->le_register_dm->text());
    settings.setValue("register/set_em", m_ui_ptr->le_register_em->text());
    settings.setValue("register/set_ss", m_ui_ptr->le_register_ss->text());
    settings.setValue("register/set_sd", m_ui_ptr->le_register_sd->text());
    settings.setValue("register/set_te", m_ui_ptr->le_register_te->text());
    settings.setValue("register/set_iterations", m_ui_ptr->le_register_iterations->text());
    settings.setValue("register/set_rg", m_ui_ptr->le_register_rg->text());
    settings.setValue("register/set_tgx", m_ui_ptr->le_register_tgx->text());
    settings.setValue("register/set_tgy", m_ui_ptr->le_register_tgy->text());
    settings.setValue("register/set_tgz", m_ui_ptr->le_register_tgz->text());
    settings.setValue("register/set_filter_x", m_ui_ptr->le_register_filter_x->text());
    settings.setValue("register/set_filter_y", m_ui_ptr->le_register_filter_y->text());
    settings.setValue("register/set_filter_z", m_ui_ptr->le_register_filter_z->text());
}

void Ponnector_Settings::on_pushButton_clicked()
{
    QString output_path = QFileDialog::getExistingDirectory (this,
                                                             tr("Select the output path."),
                                                             m_ui_ptr->_le_default_output->text(),
                                                             QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    m_ui_ptr->_le_default_output->setText(output_path);
}

void Ponnector_Settings::on_pushButton_2_clicked()
{
    QString input_path = QFileDialog::getExistingDirectory (this,
                                                            tr("Select the input path."),
                                                            m_ui_ptr->_le_default_input->text(),
                                                            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    m_ui_ptr->_le_default_input->setText(input_path);
}
