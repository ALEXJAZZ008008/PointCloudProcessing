#include "ponnector.h"

Ponnector::Ponnector(QDialog *parent):
    QDialog(parent),
    m_ui_ptr(new Ui::Ponnector)
{

}

Ponnector::~Ponnector()
{
    destructor(true);
}

Ponnector::Ponnector(Ponnector &ponnector_ref):
    m_ui_ptr(ponnector_ref.get_ui_ptr())
{

}

Ponnector & Ponnector::operator = (Ponnector &ponnector_ref)
{
    m_ui_ptr = ponnector_ref.get_ui_ptr();

    return *this;
}

Ponnector::Ponnector(Ponnector &&ponnector_ref_ref):
    m_ui_ptr(ponnector_ref_ref.get_ui_ptr())
{

}

Ponnector & Ponnector::operator = (Ponnector &&ponnector_ref_ref)
{
    m_ui_ptr = ponnector_ref_ref.get_ui_ptr();

    return *this;
}

int Ponnector::ponnector_main()
{
    m_ui_ptr->setupUi(this);

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

    return 1;
}

void Ponnector::on__psh_connec_clicked()
{
    QFileDialog::getExistingDirectory(this,
                                      tr("Select the output path."),
                                      "",
                                      QFileDialog::DontResolveSymlinks);
}
