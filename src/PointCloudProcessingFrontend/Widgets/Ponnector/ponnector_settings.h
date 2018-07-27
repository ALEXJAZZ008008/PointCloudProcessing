#ifndef PONNECTOR_SETTINGS_H
#define PONNECTOR_SETTINGS_H

#include <QDialog>
#include <QSettings>
#include <QStandardPaths>
#include <QFileDialog>

#include "ui_ponnector_settings.h"

using namespace std;

namespace Ui
{
class Ponnector_Settings;
}

class Ponnector_Settings : public QDialog
{
    Q_OBJECT

public:
    explicit Ponnector_Settings(QWidget *parent = nullptr);
    ~Ponnector_Settings();

    Ponnector_Settings(Ponnector_Settings &);
    Ponnector_Settings & operator = (Ponnector_Settings &);
    Ponnector_Settings(Ponnector_Settings &&);
    Ponnector_Settings & operator = (Ponnector_Settings &&);

    inline Ui::Ponnector_Settings * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    inline int set_ui_ptr(Ui::Ponnector_Settings *ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    int ponnector_settings_main();

    int ponnector_settings_kill(bool);

private:
    Ui::Ponnector_Settings *m_ui_ptr;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);

private slots:
    void on_buttonBox_accepted();

    void on_pushButton_clicked();
};

#endif // PONNECTOR_SETTINGS_H
