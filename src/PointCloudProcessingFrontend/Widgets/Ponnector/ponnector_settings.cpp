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

    if(settings.contains("register/filter_x"))
    {
        m_ui_ptr->le_register_filter_x->setText(settings.value("register/filter_x").toString());
    }
    else
    {
        m_ui_ptr->le_register_filter_x->setText("10.0");
    }

    if(settings.contains("register/filter_y"))
    {
        m_ui_ptr->le_register_filter_y->setText(settings.value("register/filter_y").toString());
    }
    else
    {
        m_ui_ptr->le_register_filter_y->setText("10.0");
    }

    if(settings.contains("register/filter_z"))
    {
        m_ui_ptr->le_register_filter_z->setText(settings.value("register/filter_z").toString());
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
    settings.setValue("register/set_tr_txt", m_ui_ptr->_chk_register_tr_txt->isChecked());
    settings.setValue("register/set_tr_bin", m_ui_ptr->_chk_register_tr_bin->isChecked());
    settings.setValue("register/set_icp", m_ui_ptr->rb_register_icp->isChecked());
    settings.setValue("register/set_ndt", m_ui_ptr->rb_register_ndt->isChecked());
    settings.setValue("register/filter_x", m_ui_ptr->le_register_filter_x->text());
    settings.setValue("register/filter_y", m_ui_ptr->le_register_filter_y->text());
    settings.setValue("register/filter_z", m_ui_ptr->le_register_filter_z->text());
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
