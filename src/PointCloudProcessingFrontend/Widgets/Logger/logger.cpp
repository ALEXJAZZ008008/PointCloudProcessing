#include "logger.h"
#include "ui_logger.h"

PCPLLogger::PCPLLogger(QWidget *parent) :
    QWidget(parent),
    m_ui_ptr(new Ui::Logger())
{
    m_ui_ptr->setupUi(this);
}

PCPLLogger::~PCPLLogger()
{

}

PCPLLogger::PCPLLogger(PCPLLogger &logger_ref):
    m_ui_ptr(logger_ref.get_ui_ptr())
{

}

PCPLLogger & PCPLLogger::operator = (PCPLLogger &logger_ref)
{
    m_ui_ptr = logger_ref.get_ui_ptr();

    return *this;
}

PCPLLogger::PCPLLogger(PCPLLogger &&logger_ref_ref):
    m_ui_ptr(logger_ref_ref.get_ui_ptr())
{

}

PCPLLogger & PCPLLogger::operator = (PCPLLogger &&logger_ref_ref)
{
    m_ui_ptr = logger_ref_ref.get_ui_ptr();

    return *this;
}

int PCPLLogger::logger_main()
{
    return 1;
}

int PCPLLogger::logger_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int PCPLLogger::print(string &string)
{
    m_ui_ptr->_lbl_output_msg->insertPlainText(string.c_str());

    return 1;
}

int PCPLLogger::destructor(bool hard)
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
