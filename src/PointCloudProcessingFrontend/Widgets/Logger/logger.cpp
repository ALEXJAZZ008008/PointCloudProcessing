#include "logger.h"
#include "ui_logger.h"

PCLPLogger::PCLPLogger(QWidget *parent) :
    QWidget(parent),
    m_ui_ptr(new Ui::Logger())
{
    m_ui_ptr->setupUi(this);
}

PCLPLogger::~PCLPLogger()
{

}

PCLPLogger::PCLPLogger(PCLPLogger &logger_ref):
    m_ui_ptr(logger_ref.get_ui_ptr())
{

}

PCLPLogger & PCLPLogger::operator = (PCLPLogger &logger_ref)
{
    m_ui_ptr = logger_ref.get_ui_ptr();

    return *this;
}

PCLPLogger::PCLPLogger(PCLPLogger &&logger_ref_ref):
    m_ui_ptr(logger_ref_ref.get_ui_ptr())
{

}

PCLPLogger & PCLPLogger::operator = (PCLPLogger &&logger_ref_ref)
{
    m_ui_ptr = logger_ref_ref.get_ui_ptr();

    return *this;
}

int PCLPLogger::logger_main()
{
    return 1;
}

int PCLPLogger::logger_kill(bool hard)
{
    destructor(hard);

    return 1;
}

int PCLPLogger::print(string &string)
{
    m_ui_ptr->_lbl_output_msg->insertPlainText(string.c_str());

    return 1;
}

int PCLPLogger::destructor(bool hard)
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
