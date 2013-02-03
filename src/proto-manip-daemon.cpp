/*

    Please make note: This is only a prototype for a
    manipulator daemon and is meant for demonstration
    purposes ONLY. The final version will be a robust
    state machine which monitors the arms, handles
    manipulation-related tasks, and makes sure that
    everything behaves sanely. 

*/





#include "manip.h"

int main( int argc, char **argv )
{
    Hubo_Tech hubo("proto-manip-daemon");

    ach_channel_t chan_manip_cmd;

    int r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    hubo_manip_cmd manip;
    memset( &manip, 0, sizeof(manip) );

    hubo.update();

    Eigen::Isometry3d Br, Bl;
    Vector6d right, left, zeros; zeros.setZero();
    Vector3d rtrans, ltrans, langles, rangles;
    

    hubo.getRightArmAngles(right);
    hubo.getLeftArmAngles(left);

    hubo.huboArmFK( Br, right, RIGHT );
    hubo.huboArmFK( Bl, left, LEFT );
    
    std::cout << "Performed initial FK" << std::endl;

    for(int i=0; i<3; i++)
    {
        manip.translation[RIGHT][i] = Br(i,3);
        manip.translation[LEFT][i] = Bl(i,3);
    }

    std::cout << "Putting first transformation" << std::endl;
    ach_put( &chan_manip_cmd, &manip, sizeof(manip) );

    size_t fs;

    std::cout << "About to start loop" << std::endl;

    while( !daemon_sig_quit )
    {
        hubo.update();
        ach_get( &chan_manip_cmd, &manip, sizeof(manip), &fs, NULL, ACH_O_LAST );
        
        for(int i=0; i<3; i++)
        {
            rtrans(i) = manip.translation[RIGHT][i];
            ltrans(i) = manip.translation[LEFT][i];
            rangles(i) = manip.eulerAngles[RIGHT][i];
            langles(i) = manip.eulerAngles[LEFT][i];
        }

        // Handle the right arm
        Br = Eigen::Matrix4d::Identity();
        Br.translate(rtrans);
        Br.rotate( Eigen::AngleAxisd(rangles(0), Vector3d(1,0,0)) );
        Br.rotate( Eigen::AngleAxisd(rangles(1), Vector3d(0,1,0)) );
        Br.rotate( Eigen::AngleAxisd(rangles(2), Vector3d(0,0,1)) );
        hubo.huboArmIK( right, Br, zeros, RIGHT );
        hubo.setRightArmAngles( right );

        // Handle the left arm
        Bl = Eigen::Matrix4d::Identity();
        Bl.translate(ltrans);
        Bl.rotate( Eigen::AngleAxisd(langles(0), Vector3d(1,0,0)) );
        Bl.rotate( Eigen::AngleAxisd(langles(1), Vector3d(0,1,0)) );
        Bl.rotate( Eigen::AngleAxisd(langles(2), Vector3d(0,0,1)) ); 
        hubo.huboArmIK( left, Bl, zeros, LEFT );
        hubo.setLeftArmAngles( left );

        // Send commands off to the control daemon
        hubo.sendControls();
    }
    



    ach_close( &chan_manip_cmd );




}
