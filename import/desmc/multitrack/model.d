module desmc.multitrack.model;

public import desmc.core;

/+ классифицирует по группам скелеты с разных трекеров 
   на выходе массив из групп скелетов, относящихся 
   к одному пользователю +/
interface Classifier
{
    Skeleton[][] opCall( in Skeleton[][] );
}

/+ комплексирует информацию в группах для построения 
   оптимальной оценки скелета пользователя +/
interface Complexer
{
    Skeleton[] opCall( in Skeleton[][] );
}

interface UserHandler
{
    @property
    {
        bool updated() const;
        ref const(User) user() const;
        bool isOverdue() const;
    }
    void setOverdue();
    void setSkeleton( in Skeleton s );

    /+ вероятность того, что переданный 
       скелет является трасформацией имеющегося +/
    float calcTransformPossibility( in Skeleton s ) const;
}

/+ распределяет скелеты по userhandler'ам
   возвращает нераспределённые +/
interface Destributor
{
    Skeleton[] opCall( UserHandler[], in Skeleton[] );
}

interface MultiTrackerFactory
{
    @property
    {
        Classifier classifier();
        Complexer complexer();
        Destributor destributor();
    }

    UserHandler newUserHandler( User );
}