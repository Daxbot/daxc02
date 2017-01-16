/*
 * mt9m021_mode_tbls.h - mt9m021 sensor mode tables
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MT9M021_TABLES__
#define __MT9M021_TABLES__

#include <media/camera_common.h>

enum {
	MT9M021_DEFAULT_MODE
};

static const int mt9m021_60fps[] = {
	60,
};

static const struct camera_common_frmfmt mt9m021_frmfmt[] = {
	{{1280, 960},	mt9m021_60fps,	1, 0,	MT9M021_DEFAULT_MODE},
};
#endif  /* __MT9M021_TABLES__ */

