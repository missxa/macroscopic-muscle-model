% MTC_MODEL_MATLAB calculates the force of a muscle tendon complex
% depending on the length of the contractile element, the mtc length,
% mtc contraction velocity, activity, and its parameters:
% 
% [F_MTC, dot_l_CE, F_elements] 
%               = mtc_model_matlab(l_CE, l_MTC, dot_l_MTC, q, mus_Param)
%
% It returns the muscle tendon complex force F_MTC, the contraction
% velocity of the contractile element CE, and the forces of the elements
% F_elements = [F_SEE F_PEE F_isom F_CE F_SDE]'
%
%
% Revision 1.23, 25.02.2014
%
% If you use this model for scientific purposes, please cite our article:
% D.F.B. Haeufle, M. G�nther, A. Bayer, S. Schmitt (2014) Hill-type muscle
% model with serial damping and eccentric force-velocity relation Journal
% of Biomechanics http://dx.doi.org/10.1016/j.jbiomech.2014.02.009


% Copyright (c) 2014 belongs to D. Haeufle, M. Guenther, A. Bayer, and S.
% Schmitt 
% All rights reserved. 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%  1 Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer. 
%  2 Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the distribution.
%  3 Neither the name of the owner nor the names of its contributors may be
%    used to endorse or promote products derived from this software without
%    specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
% THE POSSIBILITY OF SUCH DAMAGE.

function [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, dot_l_CE, delta_l_SEE, q, mus_Param)
% l_CE = mus_Param.CE.l_CEopt;
eps = 1e-04;
% Isometric force (Force length relation)
if l_CE >= mus_Param.CE.l_CEopt %descending branch
    F_isom = exp( - ( abs( ((l_CE/mus_Param.CE.l_CEopt)-1)/mus_Param.CE.DeltaW_limb_des ) )^mus_Param.CE.v_CElimb_des );
else %ascending branch
    F_isom = exp( - ( abs( ((l_CE/mus_Param.CE.l_CEopt)-1)/mus_Param.CE.DeltaW_limb_asc ) )^mus_Param.CE.v_CElimb_asc );
end

% Force of the parallel elastic element PEE
if l_CE >= mus_Param.PEE.l_PEE0
    F_PEE = mus_Param.PEE.K_PEE*(l_CE-mus_Param.PEE.l_PEE0)^(mus_Param.PEE.v_PEE);
else % shorter than slack length
    F_PEE = 0;
end
% F_PEE = 0;

if delta_l_SEE > eps
    F_SEE = mus_Param.SEE.DeltaF_SEE0 + mus_Param.SEE.KSEEl * delta_l_SEE;
else
    F_SEE = 0;
end

% Force of the serial elastic element SEE
%l_SEE = abs(l_MTC-l_CE); % SEE length
% if l_SEE - mus_Param.SEE.l_SEE0 > eps && l_SEE - mus_Param.SEE.l_SEEnll < -eps %non-linear part
%     F_SEE = mus_Param.SEE.KSEEnl*((l_SEE - mus_Param.SEE.l_SEE0)^(mus_Param.SEE.v_SEE));
% elseif l_SEE - mus_Param.SEE.l_SEEnll >= eps %linear part
%     F_SEE = mus_Param.SEE.DeltaF_SEE0 + mus_Param.SEE.KSEEl * delta_l_SEE; % (l_SEE - mus_Param.SEE.l_SEEnll);
% else %salck length
%     F_SEE = 0;
% end


% Hill Parameters concentric contraction
if l_CE - mus_Param.CE.l_CEopt < -eps
    A_rel=1;
else
    A_rel=F_isom;
end
A_rel = A_rel * mus_Param.CE.A_rel0*1/4*(1+3*q);

B_rel = mus_Param.CE.B_rel0*1*1/7*(3+4*q);


% no SDE and PEE
% dot_l_CE = B_rel*mus_Param.CE.l_CEopt*(1 - (mus_Param.CE.F_max*(q*F_isom + A_rel))/(F_SEE + A_rel*mus_Param.CE.F_max));

% no SDE
%dot_l_CE = B_rel*mus_Param.CE.l_CEopt*(1 - mus_Param.CE.F_max*(q*F_isom + A_rel)/(F_SEE - F_PEE + A_rel*mus_Param.CE.F_max));

% in case of an eccentric contraction:
if dot_l_CE > 0
    % calculate new Hill-parameters (asymptotes of the hyperbola)
    B_rel = (q*F_isom*(1-mus_Param.CE.F_eccentric)/(q*F_isom+A_rel)*B_rel/mus_Param.CE.S_eccentric);
    A_rel = -mus_Param.CE.F_eccentric*q*F_isom;
%     dot_l_CE = B_rel*mus_Param.CE.l_CEopt*(1 - mus_Param.CE.F_max*(q*F_isom + A_rel)/(F_SEE + A_rel*mus_Param.CE.F_max));
end

% Contractile element force
F_CE = mus_Param.CE.F_max*(( (q*F_isom+A_rel) / (1 - dot_l_CE/(mus_Param.CE.l_CEopt*B_rel) ) )-A_rel);

F_MTC = F_CE + F_PEE;

%F_MTC = F_SEE+F_SDE;

% Output the forces of the elements (for debugging/curiosity)
F_elements = [F_SEE F_PEE F_isom F_CE]';
