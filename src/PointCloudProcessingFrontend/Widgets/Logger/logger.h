#ifndef PCPLLOGGER_H
#define PCPLLOGGER_H

#include <QWidget>

#include <string>

using namespace std;

//! Used by Qt GUI
namespace Ui
{
class Logger;
}

class PCPLLogger : public QWidget
{
public:

    //! Constructor
    explicit PCPLLogger(QWidget *parent = nullptr);

    //! Destructor
    ~PCPLLogger();

    //! Copy and move constructos and assignment opperators,
    PCPLLogger(PCPLLogger &);
    PCPLLogger & operator = (PCPLLogger &);
    PCPLLogger(PCPLLogger &&);
    PCPLLogger & operator = (PCPLLogger &&);

    //! Gets the ui ptr
    inline Ui::Logger * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    //! Sets the ui ptr
    inline int set_ui_ptr(Ui::Logger * ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    //! Main
    int logger_main();

    //! Destruct remotely
    int logger_kill(bool);

    //! Prints the current log to the screen
    int print(string &);

private:

    //! Macro to indicate this is a QT object
    Q_OBJECT

    //! Holds the pointer to the ui
    Ui::Logger *m_ui_ptr;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);
};

#endif // PCPLLOGGER_H
