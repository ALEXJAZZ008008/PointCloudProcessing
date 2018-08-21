#ifndef PONNECTOR_SETTINGS_H
#define PONNECTOR_SETTINGS_H

#include <QDialog>
#include <QSettings>
#include <QStandardPaths>
#include <QFileDialog>

#include "ui_ponnector_settings.h"

using namespace std;

//! Used by Qt GUI
namespace Ui
{
class Ponnector_Settings;
}

//!
//! \class Ponnector_Settings
//! \brief The Ponnector Settings class.
//! Holds the settings for the Ponnector class
//!
class Ponnector_Settings : public QDialog
{
public:

    //! Constructor
    explicit Ponnector_Settings(QWidget *parent = nullptr);

    //! Destructor
    ~Ponnector_Settings();

    //! Copy and move constructos and assignment opperators
    Ponnector_Settings(Ponnector_Settings &);
    Ponnector_Settings & operator = (Ponnector_Settings &);
    Ponnector_Settings(Ponnector_Settings &&);
    Ponnector_Settings & operator = (Ponnector_Settings &&);

    //! Gets the ui ptr
    inline Ui::Ponnector_Settings * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    //! Sets the ui ptr
    inline int set_ui_ptr(Ui::Ponnector_Settings *ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    //! Main
    int ponnector_settings_main();

    //! Destruct remotely
    int ponnector_settings_kill(bool);

private:

    //! Macro to indicate this is a QT object
    Q_OBJECT

    //! Holds the ui pointer
    Ui::Ponnector_Settings *m_ui_ptr;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);

private slots:

    //! Event handler for Buttons
    void on_buttonBox_accepted();

    //! Event handler for Push Button button
    void on_pushButton_clicked();

    //! Event handler for Push Button 2 button
    void on_pushButton_2_clicked();
};

#endif // PONNECTOR_SETTINGS_H
