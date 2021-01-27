% Connect MatLab to Khepera via wifi
% SSH2 protocols from 
% www.uk.mathworks.com/matlabcentral/fileexchange/35409-ssh-sftp-scp-for-matlab-v2

addpath("ssh2_protocols")

%% SSH
% Open ssh2_conn connection. It will remain open until closed.
% It is best practice to return the ssh2_conn to the same variable. This
% way the connection information is always current and correct. 
% Advanced Connection: 
ssh2_conn = ssh2_config("192.168.0.101","root",".");

% Issue command and print output to screen
% ssh2_conn = ssh2_command(ssh2_conn, './khepera4_test', 1);
% ssh2_conn = ssh2_command(ssh2_conn, 'ms 50', 1);
% pause

% ssh2_conn = ssh2_command(ssh2_conn, 's', 1);
%Get file query from home directory
ssh2_conn = scp_get(ssh2_conn, "coin.jpg");

%Close connection
ssh2_conn = ssh2_close(ssh2_conn);

% %% Manual Terminal Commands
% % Open SSH connection to test machine
% command = 'ssh -tt root@192.168.0.101';
% [status, cmdout] = system(command,'-echo');
% 
% command = './khepera4_test';
% run("q")
% [status, cmdout] = system(command,'-echo');



% Close connection once tests are over
[status, cmdout] = system('exit');