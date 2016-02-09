/* This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#include "VMeDefaultExecutor.hpp"
#include "../Nav2Robot.hpp"
#include "../VMeNmpcKernel.hpp"

VMeDefaultExecutor::VMeDefaultExecutor(VMeNmpcKernel* s) {
  subjectKernel = s;
  s->attach_observer(this);
}

VMeDefaultExecutor::~VMeDefaultExecutor() {
  subjectKernel->detach_observer(this);
}

void VMeDefaultExecutor::update(Subject* s) {
  if (s == dynamic_cast<Subject*>(subjectKernel))
    commandFromLastNotify = subjectKernel->next_command();
  auto v = static_cast<VMeV*>(commandFromLastNotify.get());
}

void VMeDefaultExecutor::run(Nav2Robot& vme) {
  commandFromLastNotify->execute(vme);
}
