%HELIREAD Read a HeliLib data file
%
%   [data,header] = HELIREAD(file)
%
%   The file may either be a binary BlackBox log file (.bblog) or CSV file.
%   Should work both in Matlab and Octave.
%   Returns data as a structure and the file header.

function [data,header]=heliread(filename)
data={};

if (strcmp(filename(end-5:end),'.bblog'))
    % Binary log file
    
    fid = fopen(filename);
    filever = fread(fid,1);
    usedouble = fread(fid,1);
    if (filever ~= 1)
        error('Invalid file version number')
        return
    end
    len = fread(fid,1,'uint');
    header = [fread(fid,len,'char=>char')]';
    len = fread(fid,1,'uint');
    varline = [fread(fid,len,'char=>char')]';
    n = fread(fid,1,'uint');
    if (usedouble)
        datamat=[fread(fid,[n,inf],'double')]';
    else
        datamat=[fread(fid,[n,inf],'float')]';
    end
    for n=1:size(datamat,2)
        commaidx=find(varline==',');
        if (commaidx)
            varname=varline(1:commaidx-1);
            varline=varline(commaidx+1:end);
        else
            varname=varline;
        end
        eval(['data.' varname '=datamat(:,' num2str(n) ');']);
    end

    fclose(fid);
else
    % CSV log file
    
    % Do it differently for Octave vs Matlab
    if (exist('matlabroot'))
        csv=dlmread(filename,'\t',2,0);
    else
        csv=load(filename);
    end

    fid = fopen(filename);
    header = fgetl(fid);
    if (header(1) == '%')
        header=header(2:end);
    end

    varline = fgetl(fid);
    if (varline(1) == '%')
        varline=varline(2:end);
    end
    if (varline(end) == 13)
        varline=varline(1:end-1);
    end
    for n=1:size(csv,2)
        commaidx=find(varline==9); %tab
        if (commaidx)
            varname=varline(1:commaidx-1);
            varline=varline(commaidx+1:end);
        else
            varname=varline;
        end
        eval(['data.' varname '=csv(:,' num2str(n) ');']);
    end
end